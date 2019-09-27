package org.frcteam2910.c2019.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.frcteam2910.c2019.vision.api.Gamepiece;
import org.frcteam2910.common.robot.drivers.Limelight;
import org.frcteam2910.common.robot.subsystems.Subsystem;

public class VisionSubsystem extends Subsystem {
    private static final String CARGO_LIMELIGHT_TABLE_NAME = "limelight-cargo";
    private static final String HATCH_LIMELIGHT_TABLE_NAME = "limelight-hatch";

    private static final VisionSubsystem instance = new VisionSubsystem();

    private final Limelight cargoLimelight = new Limelight(NetworkTableInstance.getDefault().getTable(CARGO_LIMELIGHT_TABLE_NAME));
    private final Limelight hatchLimelight = new Limelight(NetworkTableInstance.getDefault().getTable(HATCH_LIMELIGHT_TABLE_NAME));

    public VisionSubsystem() {
        cargoLimelight.setCamMode(Limelight.CamMode.DRIVER);

        hatchLimelight.setCamMode(Limelight.CamMode.DRIVER);
        hatchLimelight.setPipeline(8);
    }

    public static VisionSubsystem getInstance() {
        return instance;
    }

    public Limelight getLimelight(Gamepiece gamepiece) {
        switch (gamepiece) {
            case CARGO:
                return cargoLimelight;
            case HATCH_PANEL:
                return hatchLimelight;
        }

        throw new IllegalArgumentException(String.format("Unknown gamepiece %s", gamepiece));
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    protected void initDefaultCommand() {

    }
}
