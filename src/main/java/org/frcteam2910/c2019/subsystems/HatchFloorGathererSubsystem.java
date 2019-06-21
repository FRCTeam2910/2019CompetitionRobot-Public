package org.frcteam2910.c2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2019.RobotMap;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.robot.Constants;
import org.frcteam2910.common.robot.subsystems.Subsystem;

public class HatchFloorGathererSubsystem extends Subsystem {
    public static final double CAN_UPDATE_RATE = 50.0;

    private static final double ANGLE_ENCODER_TICKS_PER_REVOLUTION = 4096.0;

    private static final double ANGLE_OFFSET_COMPETITION = Math.toRadians(-178.066);
    private static final double ANGLE_OFFSET_PRACTICE = Math.toRadians(-159.52);

    private static final double MAX_ANGLE_COMPETITION = Math.toRadians(140.5);
    private static final double MAX_ANGLE_PRACTICE = Math.toRadians(135.0);

    private static final double ANGLE_PID_P = 3.0;
    private static final double ANGLE_PID_I = 0.0075;
    private static final double ANGLE_PID_D = 10.0;
    private static final double ANGLE_PID_F = 1.0;

    private static final double ALLOWABLE_TARGET_ANGLE_ERROR = Math.toRadians(2.0); // Allowable error range of 2 degrees

    private static final double HATCH_DETECTION_DEBOUNCE_TIME = 0.5;
    private static final double INTAKE_SOFT_INTAKE_SPEED = 0.3;

    private static final HatchFloorGathererSubsystem instance = new HatchFloorGathererSubsystem();

    private final double angleOffset;
    private final double maxAngle;

    private TalonSRX angleMotor = new TalonSRX(RobotMap.HATCH_FLOOR_GATHERER_ARM_MOTOR);

    private CANSparkMax intakeMotor = new CANSparkMax(RobotMap.HATCH_FLOOR_GATHERER_FLOOR_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushed);
    private CANEncoder intakeEncoder = intakeMotor.getEncoder();

    private DigitalInput hatchDetector = new DigitalInput(4);

    private final Object canLock = new Object();
    private int angleEncoderTicks = 0;
    private double intakeRpm = 0.0;

    private Notifier canUpdateThread;

    private final Object sensorLock = new Object();
    private double currentAngle = 0.0;
    private double lastHatchDetectionTime = 0.0;
    private boolean hatchDetected = false;

    private final Object targetLock = new Object();
    private double targetAngle;
    private double targetIntakeSpeed = 0.0;

    private final Object outputLock = new Object();
    private double angleTicksOutput = 0.0;
    private double intakeOutput = 0.0;

    private HatchFloorGathererSubsystem() {
        if (Superstructure.getInstance().isPracticeBot()) {
            angleOffset = ANGLE_OFFSET_PRACTICE;
            maxAngle = MAX_ANGLE_PRACTICE;
        } else {
            angleOffset = ANGLE_OFFSET_COMPETITION;
            maxAngle = MAX_ANGLE_COMPETITION;
        }
        targetAngle = maxAngle;

        angleMotor.configFactoryDefault(Constants.CAN_TIMEOUT_MS);

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Constants.CAN_TIMEOUT_MS);
        angleMotor.setInverted(false);
        angleMotor.setSensorPhase(false);
        angleMotor.configFeedbackNotContinuous(true, Constants.CAN_TIMEOUT_MS);

        angleMotor.config_kP(0, ANGLE_PID_P, Constants.CAN_TIMEOUT_MS);
        angleMotor.config_kI(0, ANGLE_PID_I, Constants.CAN_TIMEOUT_MS);
        angleMotor.config_kD(0, ANGLE_PID_D, Constants.CAN_TIMEOUT_MS);
        angleMotor.config_kF(0, ANGLE_PID_F, Constants.CAN_TIMEOUT_MS);

        angleMotor.configAllowableClosedloopError(0, 0, Constants.CAN_TIMEOUT_MS);
        angleMotor.config_IntegralZone(0, 100, Constants.CAN_TIMEOUT_MS);

        angleMotor.setNeutralMode(NeutralMode.Brake);

        angleMotor.configForwardSoftLimitThreshold(convertAngleToEncoderTicks(0.0), Constants.CAN_TIMEOUT_MS);
        angleMotor.configForwardSoftLimitEnable(false, Constants.CAN_TIMEOUT_MS);

        angleMotor.configReverseSoftLimitThreshold(convertAngleToEncoderTicks(maxAngle), Constants.CAN_TIMEOUT_MS);
        angleMotor.configReverseSoftLimitEnable(false, Constants.CAN_TIMEOUT_MS);

        angleMotor.configMotionCruiseVelocity(5000, Constants.CAN_TIMEOUT_MS);
        angleMotor.configMotionAcceleration(1500, Constants.CAN_TIMEOUT_MS);

        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(20);

        canUpdateThread = new Notifier(() -> {
            int angleTicks = angleMotor.getSelectedSensorPosition();

            synchronized (canLock) {
                angleEncoderTicks = angleTicks;
            }

            double intakeRpm = intakeEncoder.getVelocity();
            synchronized (canLock) {
                HatchFloorGathererSubsystem.this.intakeRpm = intakeRpm;
            }

            double angleTicksOutput;
            double intakeOutput;

            synchronized (outputLock) {
                angleTicksOutput = this.angleTicksOutput;
                intakeOutput = this.intakeOutput;
            }

            angleMotor.set(ControlMode.MotionMagic, angleTicksOutput);
            intakeMotor.set(intakeOutput);
        });
        canUpdateThread.startPeriodic(1.0 / CAN_UPDATE_RATE);
    }

    private int convertAngleToEncoderTicks(double angle) {
        double unoffsettedAngle = angle - angleOffset;
        unoffsettedAngle = 2.0 * Math.PI - unoffsettedAngle;
        unoffsettedAngle %= 2.0 * Math.PI;
        if (unoffsettedAngle < 0.0) {
            unoffsettedAngle += 2.0 * Math.PI;
        }

        return (int) (unoffsettedAngle * (ANGLE_ENCODER_TICKS_PER_REVOLUTION / (2.0 * Math.PI)));
    }

    private double convertEncoderTicksToAngle(int ticks) {
        double revolutions = ticks / ANGLE_ENCODER_TICKS_PER_REVOLUTION;
        double unoffsettedAngle = 2.0 * Math.PI * revolutions;
        unoffsettedAngle = 2.0 * Math.PI - unoffsettedAngle;
        double angle = unoffsettedAngle + angleOffset;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public static HatchFloorGathererSubsystem getInstance() {
        return instance;
    }

    public double getMaxAngle() {
        return maxAngle;
    }

    public double getCurrentAngle() {
        synchronized (sensorLock) {
            return currentAngle;
        }
    }

    public double getIntakeRpm() {
        synchronized (canLock) {
            return intakeRpm;
        }
    }

    public boolean hasHatchPanel() {
        double localLastHatchDetectionTime;
        boolean localHatchDetected;
        synchronized (sensorLock) {
            localLastHatchDetectionTime = lastHatchDetectionTime;
            localHatchDetected = hatchDetected;
        }

        double deltaTime = Timer.getFPGATimestamp() - localLastHatchDetectionTime;

        return localHatchDetected || deltaTime < HATCH_DETECTION_DEBOUNCE_TIME;
    }

    public boolean isAtBottomOfTravel() {
        return getCurrentAngle() <= 0.0;
    }

    public boolean isAtTopOfTravel() {
        return getCurrentAngle() >= maxAngle;
    }

    public double getTargetAngle() {
        synchronized (targetLock) {
            return targetAngle;
        }
    }

    public boolean isAtTargetAngle() {
        return MathUtils.epsilonEquals(getCurrentAngle(), getTargetAngle(), ALLOWABLE_TARGET_ANGLE_ERROR);
    }

    public void setTargetAngle(double angle) {
        angle = MathUtils.clamp(angle, 0.0, maxAngle);

        synchronized (targetLock) {
            targetAngle = angle;
        }
    }

    public double getTargetIntakeSpeed() {
        return targetIntakeSpeed;
    }

    public void setTargetIntakeSpeed(double speed) {
        speed = MathUtils.clamp(speed, -1.0, 1.0);

        synchronized (targetLock) {
            targetIntakeSpeed = speed;
        }
    }

    private void updateSensors(double time) {
        int angleEncoderTicks;
        synchronized (canLock) {
            angleEncoderTicks = this.angleEncoderTicks;
        }

        double angle = convertEncoderTicksToAngle(angleEncoderTicks);

        // Hatch detector is normally on
        boolean localHatchDetected = !hatchDetector.get();
        synchronized (sensorLock) {
            currentAngle = angle;
            hatchDetected = localHatchDetected;
            if (localHatchDetected) {
                lastHatchDetectionTime = time;
            }
        }
    }

    @Override
    public void updateKinematics(double timestamp) {
        updateSensors(timestamp);

        double targetAngle;
        double targetIntakeSpeed;

        synchronized (targetLock) {
            targetAngle = this.targetAngle;
            targetIntakeSpeed = this.targetIntakeSpeed;
        }

        // If we have a hatch panel and we want to be either intaking or stopped, soft intake
        if (hasHatchPanel() && targetIntakeSpeed >= 0.0) {
            targetIntakeSpeed = Math.max(targetIntakeSpeed, INTAKE_SOFT_INTAKE_SPEED);
        }

        double targetAngleTicks = convertAngleToEncoderTicks(targetAngle);

        synchronized (outputLock) {
            angleTicksOutput = targetAngleTicks;
            intakeOutput = targetIntakeSpeed;
        }
    }

    @Override
    public void outputToSmartDashboard() {
        double currentAngle = getCurrentAngle();
        double targetAngle = getTargetAngle();

        SmartDashboard.putNumber("Floor Intake Arm Angle", Math.toDegrees(currentAngle));
        SmartDashboard.putNumber("Floor Intake Arm Angle Ticks", angleEncoderTicks);

        SmartDashboard.putNumber("Floor Intake Arm Target Angle", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("Floor Intake Arm Target Angle Ticks", convertAngleToEncoderTicks(targetAngle));

        SmartDashboard.putNumber("Floor Intake Target Speed", getTargetIntakeSpeed());
        SmartDashboard.putNumber("Floor Intake RPM", getIntakeRpm());
        SmartDashboard.putBoolean("Floor Intake Detects Hatch Panel", hasHatchPanel());

        SmartDashboard.putNumber("Floor Intake Angle Error", angleMotor.getClosedLoopError());

        SmartDashboard.putBoolean("Floor Intake Hatch Detected", hasHatchPanel());
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
