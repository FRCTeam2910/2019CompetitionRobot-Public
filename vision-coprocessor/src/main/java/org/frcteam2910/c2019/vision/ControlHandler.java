package org.frcteam2910.c2019.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.frcteam2910.c2019.vision.api.*;
import org.frcteam2910.c2019.vision.drivers.Limelight;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class ControlHandler implements Runnable {
    private static final Logger LOGGER = new Logger(ControlHandler.class);

    private static final String CARGO_LIMELIGHT_TABLE_NAME = "limelight-cargo";
    private static final String HATCH_LIMELIGHT_TABLE_NAME = "limelight-hatch";

    private static final ITrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
            new MaxVelocityConstraint(12.0 * 12.0),
            new MaxAccelerationConstraint(5.0 * 12.0),
            new FeedforwardConstraint(0.8, 1.0 / (14.0 * 12.0), 1.0 / (5.0 * 12.0), 0.1)
    };

    private VisionServer<Object, Object> server;

    private Map<Gamepiece, Limelight> limelights = new HashMap<>();
    private Map<Gamepiece, VisionTargetingPipeline> pipelines = new HashMap<>();

    public ControlHandler() throws IOException {
        this.server = new VisionServer<>(5800);

        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable cargoTable = instance.getTable(CARGO_LIMELIGHT_TABLE_NAME);
        NetworkTable hatchTable = instance.getTable(HATCH_LIMELIGHT_TABLE_NAME);

        limelights.put(Gamepiece.CARGO, new Limelight(cargoTable));
        limelights.put(Gamepiece.HATCH_PANEL, new Limelight(hatchTable));

        pipelines.put(Gamepiece.CARGO, new VisionTargetingPipeline2019(limelights.get(Gamepiece.CARGO)));
        pipelines.put(Gamepiece.HATCH_PANEL, new VisionTargetingPipeline2019(limelights.get(Gamepiece.HATCH_PANEL)));
    }

    @Override
    public void run() {
        while (!Thread.interrupted() && server.getSocket().isBound()) {
            try (VisionClient<Object, Object> connection = server.accept()) {
                while (!Thread.interrupted() && connection.getSocket().isConnected()) {
                    Object object = connection.receivePacket();

                    if (object instanceof ConfigCoprocessorPacket) {
                        // TODO: Config coprocessor
                        ConfigCoprocessorPacket packet = (ConfigCoprocessorPacket) object;
                        LOGGER.info("Setting %s to %s", packet.getGamepiece(), packet.isDriverMode());
                    } else if (object instanceof TrajectoryRequestPacket) {
                        // TODO: Generate trajectory
                        TrajectoryRequestPacket packet = (TrajectoryRequestPacket) object;

                        VisionTargetingPipeline pipeline = pipelines.get(packet.getGamepiece());

                        try {
                            RigidTransform2 relativePosition = pipeline.getTranslation();

                            RobotStateEstimator.State lastState = RobotStateEstimator.estimateState(packet.getTimestamp());

                            RigidTransform2 goalPose = lastState.getPose().transformBy(relativePosition);
                            goalPose = goalPose.transformBy(new RigidTransform2(new Vector2(-12.0, 0.0), Rotation2.ZERO));

                            System.out.printf("%s -> %s%n", lastState.getPose(), goalPose);

                            SplinePathGenerator generator = new SplinePathGenerator();
                            generator.setFitCheckEpsilon(1e-1);
                            Path path = generator.generate(
                                    new Waypoint(lastState.getPose().translation, lastState.getVelocity().getAngle(),
                                            lastState.getPose().rotation),
                                    new Waypoint(goalPose.translation, goalPose.rotation, goalPose.rotation)
                            );
                            Trajectory trajectory = new Trajectory(path, TRAJECTORY_CONSTRAINTS);

                            connection.sendPacket(trajectory);
                        } catch (Exception e) {
                            // Didn't get a good result
                            connection.sendPacket(e);
                            LOGGER.error(e.getMessage());
                        }
                    } else {
                        LOGGER.warn("Got packet with unknown type: %s", object.getClass().getName());
                    }
                }
            } catch (IOException e) {
                LOGGER.error(e);
            }
        }
    }
}
