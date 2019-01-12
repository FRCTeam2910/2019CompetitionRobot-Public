package org.frcteam2910.c2019.subsystems;

import org.frcteam2910.common.robot.subsystems.Subsystem;

public class CargoGrabberSubsystem extends Subsystem {
    private static final CargoGrabberSubsystem instance = new CargoGrabberSubsystem();

    private CargoGrabberSubsystem() {

    }

    public static CargoGrabberSubsystem getInstance() {
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
