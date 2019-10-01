package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2019.Robot;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2019.subsystems.HatchPlacerSubsystem;
import org.frcteam2910.c2019.subsystems.Superstructure;
import org.frcteam2910.c2019.subsystems.VisionSubsystem;
import org.frcteam2910.c2019.vision.api.Gamepiece;
import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Limelight;
import org.frcteam2910.common.util.MovingAverage;

import java.util.function.BiFunction;
import java.util.function.Function;

import static org.frcteam2910.common.robot.Utilities.deadband;

public class ImprovedVisionPlaceCommand extends Command {
    private static final boolean USE_HIGH_RESOLUTION_MODE = false;

    private static final double PLACEMENT_DISTANCE = 20.0;
    private static final double OUTER_PLACEMENT_DISTANCE = 36 + PLACEMENT_DISTANCE;
    private static final double MAX_PLACEMENT_DISTANCE_CORRECTION = 5;
    private static final double INTERPOLATION_RATIO = MAX_PLACEMENT_DISTANCE_CORRECTION / (OUTER_PLACEMENT_DISTANCE - PLACEMENT_DISTANCE);

    private static final double HORIZONTAL_CORRECTION_MAX = 4;
    private static final double HORIZONTAL_CORRECTION_FACTOR = 0.25;
    private static final double HORIZONTAL_CORRECTION_DEADBAND_BUFFER = 0.5;

    private static final int FINISH_SUCCESSES_NEEDED = 4;

    private static final int HIGH_RESOLUTION_HEIGHT = 720;
    private static final int LOW_RESOLUTION_HEIGHT = 240;

    private static final int HIGH_RESOLUTION_PIPELINE = 9;
    private static final int LOW_RESOLUTION_PIPELINE = 8;

    private Limelight limelight = VisionSubsystem.getInstance().getLimelight(Gamepiece.HATCH_PANEL);

    private final BiFunction<Double, Double, Boolean> finishCondition;
    private final Function<Rotation2, Rotation2> chooseTargetAngleFunction;
    private final ITrajectoryConstraint[] trajectoryConstraints;
    private final Vector2 targetOffset;


    private Rotation2 targetAngle;
    private Vector2 robotOrientedPositionOffset;
    private double distance = Double.POSITIVE_INFINITY;
    private double previousDistance;
    private Vector2 previousRobotOrientedPositionOffset;

    private int successesNeeded;

    public ImprovedVisionPlaceCommand(BiFunction<Double, Double, Boolean> finishCondition,
                                      Function<Rotation2, Rotation2> chooseTargetAngleFunction,
                                      ITrajectoryConstraint[] trajectoryConstraints) {
        this(finishCondition, chooseTargetAngleFunction, trajectoryConstraints,
                new Vector2(PLACEMENT_DISTANCE, 0.0));
    }

    public ImprovedVisionPlaceCommand(BiFunction<Double, Double, Boolean> finishCondition,
                                      Function<Rotation2, Rotation2> chooseTargetAngleFunction,
                                      ITrajectoryConstraint[] trajectoryConstraints,
                                      Vector2 targetOffset) {
        this.finishCondition = finishCondition;
        this.chooseTargetAngleFunction = chooseTargetAngleFunction;
        this.trajectoryConstraints = trajectoryConstraints;
        this.targetOffset = targetOffset;

        requires(DrivetrainSubsystem.getInstance());

        this.setRunWhenDisabled(true);
    }

    @Override
    protected void initialize() {
        limelight.setCamMode(Limelight.CamMode.VISION);
        if (USE_HIGH_RESOLUTION_MODE) {
            limelight.setPipeline(HIGH_RESOLUTION_PIPELINE);
        } else {
            limelight.setPipeline(LOW_RESOLUTION_PIPELINE);
        }

        distance = Double.POSITIVE_INFINITY;
        successesNeeded = FINISH_SUCCESSES_NEEDED;

        // If we don't have a hatch panel we know the target is the loading station which has an angle of
        targetAngle = chooseTargetAngleFunction.apply(Superstructure.getInstance().getGyroscope().getAngle());
    }

    @Override
    protected void execute() {
        Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);

        if (!limelight.hasTarget()) {
            DrivetrainSubsystem.getInstance().setSnapRotation(targetAngle.toRadians());
            // We don't know where the target should be
            DrivetrainSubsystem.getInstance().holonomicDrive(
                    new Vector2(
                            Robot.getOi().primaryController.getLeftYAxis().get(true),
                            Robot.getOi().primaryController.getLeftXAxis().get(true)
                    ),
                    Robot.getOi().primaryController.getRightXAxis().get(true),
                    true
            );
        } else {
            // We can see a target
            {
                double theta;
                if (USE_HIGH_RESOLUTION_MODE) {
                    theta = limelight.getTable().getEntry("tvert").getDouble(0.0) * Math.toRadians(49.7) / HIGH_RESOLUTION_HEIGHT;
                } else {
                    theta = limelight.getTable().getEntry("tvert").getDouble(0.0) * Math.toRadians(49.7) / LOW_RESOLUTION_HEIGHT;
                }
                distance = 5.83 / (2 * Math.tan(theta / 2));

                robotOrientedPositionOffset = Vector2.fromAngle(Rotation2.fromRadians(limelight.getTargetPosition().x)).scale(distance);
            }

            if (distance != previousDistance || robotOrientedPositionOffset.x != previousRobotOrientedPositionOffset.x || robotOrientedPositionOffset.y != previousRobotOrientedPositionOffset.y) {
                previousDistance = distance;
                previousRobotOrientedPositionOffset = robotOrientedPositionOffset;

                double unboundedHorizontalError = Math.abs(robotOrientedPositionOffset.y) * HORIZONTAL_CORRECTION_FACTOR;
                double boundedHorizontalError = Math.min(unboundedHorizontalError, HORIZONTAL_CORRECTION_MAX);
                double horizontalError = deadband(boundedHorizontalError, HORIZONTAL_CORRECTION_DEADBAND_BUFFER);
                Vector2 newTargetOffset = targetOffset.add(interpolate(distance) + horizontalError, 0);

                SmartDashboard.putNumber("Target distance", robotOrientedPositionOffset.length);

                RigidTransform2 currentPose = new RigidTransform2(
                        DrivetrainSubsystem.getInstance().getKinematicPosition(),
                        Superstructure.getInstance().getGyroscope().getAngle()
                );

                RigidTransform2 targetPose = currentPose.transformBy(
                        new RigidTransform2(
                                robotOrientedPositionOffset.subtract(newTargetOffset).multiply(-1.0, 1.0),
                                Rotation2.ZERO
                        )
                );

                Path path = new Path(targetAngle);
                path.addSegment(
                        new PathLineSegment(
                                DrivetrainSubsystem.getInstance().getKinematicPosition(),
                                targetPose.translation
                        )
                );

                double startingVelocity = DrivetrainSubsystem.getInstance().getKinematicVelocity().length;
                Trajectory.Segment lastSegment = DrivetrainSubsystem.getInstance().getFollower().getLastSegment();
                if (lastSegment != null) {
                    startingVelocity = lastSegment.velocity;
                }

                Trajectory trajectory = new Trajectory(
                        startingVelocity,
                        0.0 * 12.0,
                        path,
                        trajectoryConstraints
                );

                DrivetrainSubsystem.getInstance().getFollower().follow(trajectory);
            }
        }
    }

    @Override
    protected boolean isFinished() {
        if (robotOrientedPositionOffset == null) {
            return false;
        }

        if (finishCondition.apply(distance, robotOrientedPositionOffset.y)) {
            successesNeeded--;
        } else {
            successesNeeded = FINISH_SUCCESSES_NEEDED;
        }

        return successesNeeded == 0;
    }

    @Override
    protected void end() {
        DrivetrainSubsystem.getInstance().stop();

        DrivetrainSubsystem.getInstance().getFollower().cancel();

        Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        limelight.setCamMode(Limelight.CamMode.DRIVER);
    }

    private double interpolate(double toInterpolate) {
        if (toInterpolate <= PLACEMENT_DISTANCE) {
            return 0;
        } else if (toInterpolate >= OUTER_PLACEMENT_DISTANCE) {
            return MAX_PLACEMENT_DISTANCE_CORRECTION;
        } else {
            return INTERPOLATION_RATIO * (toInterpolate - PLACEMENT_DISTANCE);
        }
    }
}
