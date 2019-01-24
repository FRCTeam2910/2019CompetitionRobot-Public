package org.frcteam2910.c2019;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.frcteam2910.c2019.subsystems.ClimberSubsystem;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2019.subsystems.HatchSubsystem;
import org.frcteam2910.c2019.subsystems.CargoGrabberSubsystem;
import org.frcteam2910.c2019.subsystems.ArmSubsystem;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;

public class Robot extends TimedRobot {
    private static final double UPDATE_DT = 5e-3; // 5 ms

    private final SubsystemManager subsystemManager = new SubsystemManager(
            HatchSubsystem.getInstance(),
            ClimberSubsystem.getInstance(),
            DrivetrainSubsystem.getInstance(),
            CargoGrabberSubsystem.getInstance(),
            ArmSubsystem.getInstance());

    private static final OI oi = new OI();

    public Robot() {
        oi.bindButtons();
    }

    public static OI getOi() {
        return oi;
    }

    @Override
    public void robotInit() {
        subsystemManager.enableKinematicLoop(UPDATE_DT);
    }

    @Override
    public void robotPeriodic() {
        subsystemManager.outputToSmartDashboard();
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {

    }
}
