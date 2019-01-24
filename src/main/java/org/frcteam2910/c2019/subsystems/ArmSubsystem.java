package org.frcteam2910.c2019.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frcteam2910.common.robot.subsystems.Subsystem;
import org.frcteam2910.c2019.RobotMap;

public class ArmSubsystem extends Subsystem {
    private static final ArmSubsystem instance = new ArmSubsystem();

    private final CANSparkMax[] motors = {
            new CANSparkMax(RobotMap.ARM_MOTOR_A, CANSparkMaxLowLevel.MotorType.kBrushless),
            new CANSparkMax(RobotMap.ARM_MOTOR_B, CANSparkMaxLowLevel.MotorType.kBrushless)
    };

    private ArmSubsystem() {

    }

    public static ArmSubsystem getInstance() {
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
