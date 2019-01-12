package org.frcteam2910.c2019.subsystems;

import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.robot.subsystems.SwerveDrivetrain;

public class DrivetrainSubsystem extends SwerveDrivetrain {

    private static final DrivetrainSubsystem instance = new DrivetrainSubsystem();

    private DrivetrainSubsystem() {

    }

    public static DrivetrainSubsystem getInstance() {
        return instance;
    }

    @Override
    public SwerveModule[] getSwerveModules() {
        return new SwerveModule[0];
    }

    @Override
    public Gyroscope getGyroscope() {
        return null;
    }

    @Override
    public double getMaximumVelocity() {
        return 0;
    }

    @Override
    public double getMaximumAcceleration() {
        return 0;
    }

    @Override
    protected void initDefaultCommand() {

    }
}
