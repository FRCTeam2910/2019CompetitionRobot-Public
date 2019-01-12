package org.frcteam2910.c2019;

import edu.wpi.first.wpilibj.TimedRobot;
import org.frcteam2910.c2019.subsystems.ClimberSubsystem;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2019.subsystems.HatchSubsystem;
import org.frcteam2910.c2019.subsystems.CargoGrabberSubsystem;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;

public class Robot extends TimedRobot {
    private static final double UPDATE_DT = 5e-3; // 5 ms

    private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    private final HatchSubsystem hatchSubsystem = HatchSubsystem.getInstance();
    private final CargoGrabberSubsystem cargoGrabberSubsystem = CargoGrabberSubsystem.getInstance();
    private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    private final SubsystemManager subsystemManager = new SubsystemManager(hatchSubsystem, climberSubsystem, drivetrainSubsystem, cargoGrabberSubsystem);


    @Override
    public void teleopInit() {
        subsystemManager.enableKinematicLoop(UPDATE_DT);
    }

    @Override
    public void autonomousInit() {
        subsystemManager.enableKinematicLoop(UPDATE_DT);
    }

    @Override
    public void disabledInit() {
        subsystemManager.disableKinematicLoop();
    }
}
