package org.frcteam2910.c2019.subsystems;

import org.frcteam2910.common.robot.subsystems.Subsystem;

public class ClimberSubsystem extends Subsystem {

    private static final ClimberSubsystem instance = new ClimberSubsystem();

    private ClimberSubsystem() {}

    public static ClimberSubsystem getInstance() {
        return instance;
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void initDefaultCommand() {

    }
}
