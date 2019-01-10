package org.frcteam2910.c2019.subsystems;

import org.frcteam2910.common.robot.subsystems.Subsystem;

public class HatchSubsystem extends Subsystem {

    private static final HatchSubsystem instance = new HatchSubsystem();

    private HatchSubsystem() {

    }

    public static HatchSubsystem getInstance() {
        return instance;
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
