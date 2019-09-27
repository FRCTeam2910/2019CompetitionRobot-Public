package org.frcteam2910.c2019;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2019.autonomous.AutonomousSelector;
import org.frcteam2910.c2019.autonomous.AutonomousTrajectories;
import org.frcteam2910.c2019.subsystems.*;
import org.frcteam2910.c2019.vision.api.Gamepiece;
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

    private AutonomousTrajectories autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.CONSTRAINTS);
    private AutonomousSelector autonomousSelector = new AutonomousSelector(autonomousTrajectories);

    private Command autonomousCommand = null;

    public Robot() {
        oi.bindButtons(autonomousSelector);
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
        subsystemManager.outputToSmartDashboard();

        SmartDashboard.putNumber("Arm Angle",
                Math.toDegrees(CargoArmSubsystem.getInstance().getCurrentAngle()));
        SmartDashboard.putNumber("Robot Angle",
                Superstructure.getInstance().getGyroscope().getAngle().toDegrees());
        SmartDashboard.putNumber("Gyro Pitch",
                Math.toDegrees(Superstructure.getInstance().getGyroscope().getAxis(NavX.Axis.ROLL)));
        SmartDashboard.putBoolean("Is Competition Bot",
                Superstructure.getInstance().isCompetitionBot());
        SmartDashboard.putBoolean("Is Practice Bot",
                Superstructure.getInstance().isPracticeBot());
    }

    @Override
    public void teleopInit() {
//        if (autonomousCommand != null) {
//            autonomousCommand.cancel();
//            autonomousCommand = null;
//        }
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        autonomousCommand = autonomousSelector.getCommand();
        autonomousCommand.start();
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
//        if (autonomousCommand != null) {
//            autonomousCommand.cancel();
//            autonomousCommand = null;
//        }
//        Scheduler.getInstance().removeAll();
    }

    @Override
    public void disabledPeriodic() {
        boolean calibrationMode = SmartDashboard.getBoolean("Limelight Calibration Mode", false);

        Limelight.CamMode mode = calibrationMode ? Limelight.CamMode.VISION : Limelight.CamMode.DRIVER;
        VisionSubsystem.getInstance().getLimelight(Gamepiece.HATCH_PANEL).setCamMode(mode);
        VisionSubsystem.getInstance().getLimelight(Gamepiece.CARGO).setCamMode(mode);
    }
}
