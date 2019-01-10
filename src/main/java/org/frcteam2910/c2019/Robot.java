package org.frcteam2910.c2019;

import edu.wpi.first.wpilibj.TimedRobot;
import org.frcteam2910.c2019.subsystems.HatchSubsystem;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;

public class Robot extends TimedRobot {
    private static final double UPDATE_DT = 5e-3; // 5 ms

    private final HatchSubsystem hatchSubsystem = HatchSubsystem.getInstance();
    private final SubsystemManager subsystemManager = new SubsystemManager(hatchSubsystem);

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
