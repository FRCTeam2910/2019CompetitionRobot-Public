package org.frcteam2910.c2019.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import org.frcteam2910.c2019.vision.api.Gamepiece;
import org.frcteam2910.common.robot.drivers.Limelight;
import org.frcteam2910.common.robot.subsystems.Subsystem;

public class VisionSubsystem extends Subsystem {
    private static final String CARGO_LIMELIGHT_TABLE_NAME = "limelight-cargo";
    private static final String HATCH_LIMELIGHT_TABLE_NAME = "limelight-hatch";

    private static final VisionSubsystem instance = new VisionSubsystem();

    private final Limelight cargoLimelight = new Limelight(CARGO_LIMELIGHT_TABLE_NAME);
    private final Limelight hatchLimelight = new Limelight(HATCH_LIMELIGHT_TABLE_NAME);

//    private final VideoSource cargoVideoSource;
//    private final VideoSource hatchVideoSource;

//    private final VideoSink videoServerOutput;

    public VisionSubsystem() {
        cargoLimelight.setCamMode(Limelight.CamMode.DRIVER);
        cargoLimelight.setLedMode(Limelight.LedMode.ON);

        hatchLimelight.setCamMode(Limelight.CamMode.DRIVER);
        hatchLimelight.setLedMode(Limelight.LedMode.ON);

//        cargoVideoSource = new HttpCamera("Cargo Camera", String.format("%s.local:5800", CARGO_LIMELIGHT_TABLE_NAME));
//        hatchVideoSource = new HttpCamera("Hatch Camera", String.format("%s.local:5800", HATCH_LIMELIGHT_TABLE_NAME));
//        videoServerOutput = CameraServer.getInstance().addSwitchedCamera("");

        // Default to the hatch camera
//        videoServerOutput.setSource(hatchVideoSource);
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

    /**
     * Sets which camera feed is displayed on the driver station.
     *
     * @param gamepiece which gamepiece's camera to display
     */
    public void setActiveDriverCamera(Gamepiece gamepiece) {
//        switch (gamepiece) {
//            case CARGO:
//                videoServerOutput.setSource(cargoVideoSource);
//                break;
//            case HATCH_PANEL:
//                videoServerOutput.setSource(hatchVideoSource);
//        }
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
