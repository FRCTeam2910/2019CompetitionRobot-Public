package org.frcteam2910.c2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2019.commands.CargoGrabberDefaultCommand;
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
    private boolean leftCargoDetected = false;
    private double leftCargoDetectionTime = 0.0;
    private boolean rightCargoDetected = false;
    private double rightCargoDetectionTime = 0.0;
    private double topCurrent = 0.0;

    private final Notifier canThread = new Notifier(() -> {
        double localTopCurrent = topMotor.getOutputCurrent();

        double bottomSpeed;
        double topSpeed;
        synchronized (canLock) {
            bottomSpeed = bottomMotorSpeed;
            topSpeed = topMotorSpeed;
            topCurrent = localTopCurrent;
        }
        bottomMotor.set(ControlMode.PercentOutput, bottomSpeed);
        topMotor.set(topSpeed);

        synchronized (canLock) {
            // Beam break sensor is normally closed
            leftCargoDetected = bottomMotor.getSensorCollection().isFwdLimitSwitchClosed();
            if (leftCargoDetected) {
                leftCargoDetectionTime = Timer.getFPGATimestamp();
            }

            rightCargoDetected = bottomMotor.getSensorCollection().isRevLimitSwitchClosed();
            if (rightCargoDetected) {
                rightCargoDetectionTime = Timer.getFPGATimestamp();
            }
        }
    });

    private CargoGrabberSubsystem() {
        topMotor.setSmartCurrentLimit(40);
        topMotor.setInverted(true);

        bottomMotor.configFactoryDefault();
        bottomMotor.configForwardSoftLimitEnable(false);
        bottomMotor.configReverseSoftLimitEnable(false);
        bottomMotor.overrideLimitSwitchesEnable(false);
        bottomMotor.setInverted(true);

        canThread.startPeriodic(CAN_THREAD_UPDATE_DURATION);
    }

    public static CargoGrabberSubsystem getInstance() {
        return instance;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putBoolean("Cargo Detected", hasCargo());
        double localTopCurrent;
        synchronized (canLock) {
            localTopCurrent = topCurrent;
        }
        SmartDashboard.putNumber("Top Current", localTopCurrent);
    }

    @Override
    public void stop() {
        setIntakeSpeed(0);
    }

    public void setIntakeSpeed(double speed) {
        setTopIntakeSpeed(speed);
        setBottomIntakeSpeed(speed);
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

    public boolean hasCargo() {
        return hasLeftCargo() && hasRightCargo();
    }

    public boolean hasLeftCargo() {
        synchronized (canLock) {
            return leftCargoDetected || (Timer.getFPGATimestamp() - leftCargoDetectionTime) < 0.25;
        }
    }

    public boolean hasRightCargo() {
        synchronized (canLock) {
            return rightCargoDetected || (Timer.getFPGATimestamp() - rightCargoDetectionTime) < 0.25;
        }
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new CargoGrabberDefaultCommand());
    }
}
