package org.frcteam2910.c2019.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.robot.subsystems.Subsystem;
import org.frcteam2910.c2019.RobotMap;

public class ArmSubsystem extends Subsystem {
    private static final ArmSubsystem instance = new ArmSubsystem();
    private static final double ANGLE_OFFSET = Math.toRadians(-204.93); //TODO: Find Real Value
    public static final double MAX_ANGLE = Math.toRadians(112.0);
    private static final double ENCODER_GEAR_RATIO = 24.0 / 54.0;
    private static final double ALLOWABLE_TARGET_ANGLE_ERROR = Math.toRadians(2.0); // Allowable error range of 2 degrees

    private final Spark[] motors = {
            new Spark(RobotMap.ARM_MOTOR_A),
            new Spark(RobotMap.ARM_MOTOR_B)
    };

    private Object sensorLock = new Object();
    private final PidConstants pidConstants = new PidConstants(2.0, 1.0, 0.0); //TODO: Tune PID
    private AnalogInput armEncoder = new AnalogInput(RobotMap.CARGO_ARM_ENCODER_PORT);
    private PidController armController = new PidController(pidConstants);
    private double previousTimestamp = 0;
    private double currentAngle = Math.toRadians(0);

    public boolean disabled = true;

    private ArmSubsystem() {
        armController.setInputRange(0.0, 2.0 * Math.PI);
        armController.setContinuous(true);
        armController.setOutputRange(-1.0, 1.0);
        armController.setIntegralRange(Math.toRadians(10.0));

        updateSensors();

        armController.setSetpoint(getCurrentAngle());
    }

    public static ArmSubsystem getInstance() {
        return instance;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Cargo Arm Angle", Math.toDegrees(getCurrentAngle()));
        SmartDashboard.putNumber("Cargo Arm Target Angle", Math.toDegrees(getTargetAngle()));
        synchronized (sensorLock) {
            SmartDashboard.putNumber("Cargo Arm Output", motors[0].get());
        }
    }

    public boolean isDisabled() {
        synchronized (sensorLock) {
            return disabled;
        }
    }

    public void setDisabled(boolean disabled) {
        synchronized (sensorLock) {
            this.disabled = disabled;
        }
    }

    public double getCurrentAngle() {
        synchronized (sensorLock) {
            return currentAngle;
        }
    }

    public double getTargetAngle() {
        synchronized (sensorLock) {
            return armController.getSetpoint();
        }
    }

    public void setTargetAngle(double angle) {
        angle = MathUtils.clamp(angle, 0.0, MAX_ANGLE);

        synchronized (sensorLock) {
            armController.setSetpoint(angle);
        }

        setDisabled(false);
    }

    @Override
    public void updateKinematics(double timestamp) {
        double dTime;

        dTime = timestamp - previousTimestamp;
        previousTimestamp = timestamp;

        updateSensors();

        double speed;
        synchronized(sensorLock) {
            speed = armController.calculate(getCurrentAngle(), dTime);
        }

        double kF = 0.0625;
        speed += kF * Math.cos(armController.getSetpoint());

        if (isDisabled()) {
            motors[0].set(0.0);
            motors[1].set(0.0);
        } else {
            motors[0].set(speed);
            motors[1].set(speed);
        }
    }

    public boolean isWithinTargetAngleRange(double targetAngle) {
        double cAngle = getCurrentAngle();
        return MathUtils.epsilonEquals(cAngle, targetAngle, ALLOWABLE_TARGET_ANGLE_ERROR);
    }

    private void updateSensors() {
        double encoderRotations = armEncoder.getVoltage() / RobotController.getVoltage5V();
        double armRotations = ENCODER_GEAR_RATIO * encoderRotations;
        double armUnadjustedAngle = armRotations * 2.0 * Math.PI;
        double armAngle = 2.0 * Math.PI - armUnadjustedAngle;
        armAngle += ANGLE_OFFSET;
        armAngle %= 2.0 * Math.PI;
        if (armAngle < 0.0) {
            armAngle += 2.0 * Math.PI;
        }

        synchronized (sensorLock) {
            currentAngle = armAngle;
        }
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
