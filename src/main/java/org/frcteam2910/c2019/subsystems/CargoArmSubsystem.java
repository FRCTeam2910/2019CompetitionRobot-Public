package org.frcteam2910.c2019.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.subsystems.Subsystem;
import org.frcteam2910.c2019.RobotMap;

public class CargoArmSubsystem extends Subsystem {
    private static final CargoArmSubsystem instance = new CargoArmSubsystem();
    private static final double ANGLE_OFFSET = Math.toRadians(-205.3); //TODO: Find Real Value
    public static final double MAX_ANGLE = Math.toRadians(109.0);
    private static final double ENCODER_GEAR_RATIO = 24.0 / 54.0;
    private static final double ALLOWABLE_TARGET_ANGLE_ERROR = Math.toRadians(2.0); // Allowable error range of 2 degrees

    private static final double ANGLE_FEEDFORWARD = 0.0625;

    private final Spark[] motors = {
            new Spark(RobotMap.ARM_MOTOR_A),
            new Spark(RobotMap.ARM_MOTOR_B)
    };

    private final PidConstants positionPidConstants = new PidConstants(2.0, 1.0, 0.0);
    private final PidConstants pitchPidConstants = new PidConstants(1.0 / Math.toRadians(10.0), 0.0, 0.0);

    private PidController positionController = new PidController(positionPidConstants);
    private PidController pitchController = new PidController(pitchPidConstants);

    private final Object sensorLock = new Object();
    private AnalogInput angleEncoder = new AnalogInput(RobotMap.CARGO_ARM_ENCODER_PORT);
    private double currentAngle = Math.toRadians(0);

    private final Object stateLock = new Object();
    private State currentState = State.DISABLED;
    private double targetAngle = 0.0;
    private double targetPitch = 0.0;

    private double previousTimestamp = 0;

    private CargoArmSubsystem() {
        positionController.setInputRange(0.0, 2.0 * Math.PI);
        positionController.setContinuous(true);
        positionController.setOutputRange(-1.0, 1.0);
        positionController.setIntegralRange(Math.toRadians(10.0));
    }

    public static CargoArmSubsystem getInstance() {
        return instance;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Cargo Arm Angle", Math.toDegrees(getCurrentAngle()));
        SmartDashboard.putNumber("Cargo Arm Target Angle", Math.toDegrees(getTargetAngle()));
    }

    public double getCurrentAngle() {
        synchronized (sensorLock) {
            return currentAngle;
        }
    }

    public double getTargetAngle() {
        synchronized (stateLock) {
            return targetAngle;
        }
    }

    public void setTargetAngle(double angle) {
        angle = MathUtils.clamp(angle, 0.0, MAX_ANGLE);

        synchronized (stateLock) {
            targetAngle = angle;
            currentState = State.POSITION;
        }
    }

    public double getTargetPitch() {
        synchronized (stateLock) {
            return targetPitch;
        }
    }

    public void setTargetPitch(double pitch) {
        synchronized (stateLock) {
            targetPitch = pitch;
            currentState = State.PITCH;
        }
    }

    public void disable() {
        synchronized (stateLock) {
            currentState = State.DISABLED;
        }
    }

    @Override
    public void updateKinematics(double timestamp) {
        State localCurrentState;
        synchronized (stateLock) {
            localCurrentState = currentState;
        }

        double dt = timestamp - previousTimestamp;
        previousTimestamp = timestamp;

        updateSensors();

        double output = 0.0;

        double currentAngle = getCurrentAngle();

        switch (localCurrentState) {
            case DISABLED:
                output = 0.0;
                break;
            case PITCH:
                double localTargetPitch;
                synchronized (stateLock) {
                    localTargetPitch = targetPitch;
                }

                double currentPitch = Superstructure.getInstance().getGyroscope().getAxis(NavX.Axis.ROLL);

                pitchController.setSetpoint(localTargetPitch);
                output = pitchController.calculate(currentPitch, dt);

                // If we are close to the bottom of the range of travel, force the arm down.
                if (MathUtils.epsilonEquals(currentAngle, 0.0, Math.toRadians(5.0))) { // TODO: Make constant
                    output = -0.4; // TODO: Make constant
                }

                if (currentAngle > 65.0 && currentAngle < 355.0) {
                    output = MathUtils.clamp(output, -1.0, 0.0);
                }
                break;
            case POSITION:
                double localTargetAngle;
                synchronized (stateLock) {
                    localTargetAngle = targetAngle;
                }

                positionController.setSetpoint(targetAngle);
                output = positionController.calculate(currentAngle, dt);

                output += ANGLE_FEEDFORWARD * Math.cos(localTargetAngle);
        }

        motors[0].set(output);
        motors[1].set(output);
    }

    public boolean isWithinTargetAngleRange(double targetAngle) {
        double currentAngle = getCurrentAngle();
        return MathUtils.epsilonEquals(currentAngle, targetAngle, ALLOWABLE_TARGET_ANGLE_ERROR);
    }

    private void updateSensors() {
        double encoderRotations = angleEncoder.getVoltage() / RobotController.getVoltage5V();
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


    public enum State {
        DISABLED,
        POSITION,
        PITCH
    }
}
