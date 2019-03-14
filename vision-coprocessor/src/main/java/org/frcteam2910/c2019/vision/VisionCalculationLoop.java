package org.frcteam2910.c2019.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.frcteam2910.c2019.vision.api.Gamepiece;
import org.frcteam2910.c2019.vision.drivers.Limelight;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.math.RigidTransform2;

import java.util.HashMap;
import java.util.Map;

public class VisionCalculationLoop implements Runnable {
    private static final Logger LOGGER = new Logger(VisionTargetingPipeline.class);

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("vision-coprocessor");

    private static final String CARGO_LIMELIGHT_TABLE_NAME = "limelight-cargo";
    private static final String HATCH_LIMELIGHT_TABLE_NAME = "limelight-hatch";

    private Map<Gamepiece, Limelight> limelights = new HashMap<>();
    private Map<Gamepiece, VisionTargetingPipeline> pipelines = new HashMap<>();

    public VisionCalculationLoop() {
        limelights.put(Gamepiece.CARGO, new Limelight(NetworkTableInstance.getDefault().getTable(CARGO_LIMELIGHT_TABLE_NAME)));
        limelights.put(Gamepiece.HATCH_PANEL, new Limelight(NetworkTableInstance.getDefault().getTable(HATCH_LIMELIGHT_TABLE_NAME)));

        pipelines.put(Gamepiece.CARGO, new VisionTargetingPipeline2019(limelights.get(Gamepiece.CARGO)));
        pipelines.put(Gamepiece.HATCH_PANEL, new VisionTargetingPipeline2019(limelights.get(Gamepiece.HATCH_PANEL)));
    }

    @Override
    public void run() {
        NetworkTableEntry activeEntry = table.getEntry("active");
        NetworkTableEntry cameraEntry = table.getEntry("camera");

        activeEntry.setBoolean(false);
        cameraEntry.setNumber(Gamepiece.CARGO.ordinal());

        NetworkTableEntry xEntry = table.getEntry("x");
        NetworkTableEntry yEntry = table.getEntry("y");
        NetworkTableEntry angleEntry = table.getEntry("angle");

        xEntry.setNumber(Double.NaN);
        yEntry.setNumber(Double.NaN);
        angleEntry.setNumber(Double.NaN);

        boolean lastActive = false;
        Gamepiece lastGamepiece = Gamepiece.CARGO;

        while (!Thread.interrupted()) {
            boolean active = activeEntry.getBoolean(false);

            Gamepiece gamepiece = Gamepiece.values()[cameraEntry.getNumber(0).intValue()];

            if (gamepiece != lastGamepiece) {
                limelights.get(lastGamepiece).setCamMode(Limelight.CamMode.DRIVER);
                limelights.get(lastGamepiece).setLedMode(Limelight.LedMode.OFF);
            }

            Limelight limelight = limelights.get(gamepiece);
            VisionTargetingPipeline pipeline = pipelines.get(gamepiece);
            if (active) {
                if (gamepiece != lastGamepiece || !lastActive) {
                    limelight.setCamMode(Limelight.CamMode.VISION);
                    limelight.setLedMode(Limelight.LedMode.DEFAULT);
                    LOGGER.info("Enabling vision for %s", gamepiece);
                }

                try {
                    RigidTransform2 transform = pipeline.getTranslation();

                    xEntry.setNumber(transform.translation.y);
                    yEntry.setNumber(transform.translation.x);
                    angleEntry.setNumber(transform.rotation.toRadians());
                } catch (Exception e) {
                    LOGGER.error(e);
                }
            } else if (lastActive) {
                limelight.setCamMode(Limelight.CamMode.DRIVER);
                limelight.setLedMode(Limelight.LedMode.OFF);

                xEntry.setNumber(Double.NaN);
                yEntry.setNumber(Double.NaN);
                angleEntry.setNumber(Double.NaN);
            }

            lastActive = active;
            lastGamepiece = gamepiece;
        }
    }
}
