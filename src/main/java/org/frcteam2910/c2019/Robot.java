package org.frcteam2910.c2019;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2019.commands.GotoTargetCommand;
import org.frcteam2910.c2019.subsystems.*;
import org.frcteam2910.c2019.vision.api.Gamepiece;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.drivers.Limelight;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;

public class Robot extends TimedRobot {
    private static final double UPDATE_DT = 5e-3; // 5 ms

    private final SubsystemManager subsystemManager = new SubsystemManager(
            ClimberSubsystem.getInstance(),
            DrivetrainSubsystem.getInstance(),
            CargoGrabberSubsystem.getInstance(),
            CargoArmSubsystem.getInstance(),
            HatchFloorGathererSubsystem.getInstance(),
            HatchPlacerSubsystem.getInstance(),
            VisionSubsystem.getInstance()
    );

    private static final OI oi = new OI();

    private Command autonomousCommand = null;

    public Robot() {
        oi.bindButtons();

        MjpegServer hatch = CameraServer.getInstance().addServer("limelight-hatch", 5800);
        CameraServer.getInstance().addServer("limelight-cargo", 5800);
    }

    public static OI getOi() {
        return oi;
    }

    @Override
    public void robotInit() {
        SmartDashboard.putBoolean("Limelight Calibration Mode", false);

        subsystemManager.enableKinematicLoop(UPDATE_DT);
    }

    @Override
    public void robotPeriodic() {
//        subsystemManager.outputToSmartDashboard();

        SmartDashboard.putNumber("Arm Angle",
                Math.toDegrees(CargoArmSubsystem.getInstance().getCurrentAngle()));
        SmartDashboard.putNumber("Gyro Pitch",
                Math.toDegrees(Superstructure.getInstance().getGyroscope().getAxis(NavX.Axis.ROLL)));
        SmartDashboard.putBoolean("Is Competition Bot",
                Superstructure.getInstance().isCompetitionBot());
        SmartDashboard.putBoolean("Is Practice Bot",
                Superstructure.getInstance().isPracticeBot());
    }

    @Override
    public void disabledPeriodic() {
        boolean calibrationMode = SmartDashboard.getBoolean("Limelight Calibration Mode", false);

        Limelight.CamMode mode = calibrationMode ? Limelight.CamMode.VISION : Limelight.CamMode.DRIVER;
        VisionSubsystem.getInstance().getLimelight(Gamepiece.HATCH_PANEL).setCamMode(mode);
        VisionSubsystem.getInstance().getLimelight(Gamepiece.CARGO).setCamMode(mode);
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        Superstructure.getInstance().getGyroscope().setAdjustmentAngle(
                Superstructure.getInstance().getGyroscope().getUnadjustedAngle().rotateBy(Rotation2.fromDegrees(180.0))
        );

        teleopInit();
    }

    @Override
    public void autonomousPeriodic() {
        teleopPeriodic();
    }

    @Override
    public void disabledInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
        Scheduler.getInstance().removeAll();
    }
}
