package org.frcteam2910.c2019.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frcteam2910.common.robot.subsystems.Subsystem;
import org.frcteam2910.c2019.RobotMap;

public class CargoGrabberSubsystem extends Subsystem {
    private static final CargoGrabberSubsystem instance = new CargoGrabberSubsystem();

    private final CANSparkMax topMotor = new CANSparkMax(RobotMap.GRABBER_TOP_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax bottomMotor = new CANSparkMax(RobotMap.GRABBER_BOTTOM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

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
