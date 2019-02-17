package org.frcteam2910.c2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Notifier;
import org.frcteam2910.common.robot.subsystems.Subsystem;
import org.frcteam2910.c2019.RobotMap;

public class CargoGrabberSubsystem extends Subsystem {
    private static final CargoGrabberSubsystem instance = new CargoGrabberSubsystem();

    private static final double CAN_THREAD_UPDATE_DURATION = 0.02; // 50Hz

    private final CANSparkMax topMotor = new CANSparkMax(RobotMap.GRABBER_TOP_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final TalonSRX bottomMotor = new TalonSRX(RobotMap.GRABBER_BOTTOM_MOTOR);

    private double bottomMotorSpeed = 0;
    private double topMotorSpeed = 0;

    private final Object canLock = new Object();

    private final Notifier canThread = new Notifier(() -> {
        double bottomSpeed;
        double topSpeed;
        synchronized (canLock) {
            bottomSpeed = bottomMotorSpeed;
        }
        bottomMotor.set(ControlMode.PercentOutput, bottomSpeed);
        synchronized (canLock) {
            topSpeed = topMotorSpeed;
        }
        topMotor.set(topSpeed);
    });

    private CargoGrabberSubsystem() {
        canThread.startPeriodic(CAN_THREAD_UPDATE_DURATION);
    }

    public static CargoGrabberSubsystem getInstance() {
        return instance;
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void stop() {
        setIntakeSpeed(0);
    }

    public void setIntakeSpeed(double speed) {
        setTopIntakeSpeed(speed);
        setBottomIntakeSpeed(-Math.abs(speed));
    }

    public void setTopIntakeSpeed(double speed) {
        synchronized (canLock) {
            topMotorSpeed = speed;
        }
    }

    public void setBottomIntakeSpeed(double speed) {
        synchronized (canLock) {
            bottomMotorSpeed = speed;
        }
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    protected void initDefaultCommand() {

    }
}
