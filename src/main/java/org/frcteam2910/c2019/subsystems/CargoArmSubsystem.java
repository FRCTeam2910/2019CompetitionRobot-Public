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
    public static final double CARGO_SHIP_SCORE_ANGLE = Math.toRadians(100.0);
    public static final double ROCKET_SCORE_ANGLE = Math.toRadians(63.0);
    public static final double BOTTOM_ANGLE = Math.toRadians(4.0);

    private static final double ANGLE_OFFSET_COMPETITION = Math.toRadians(-214.97);
    private static final double ANGLE_OFFSET_PRACTICE = Math.toRadians(-212.11148939808933);

    // These really shouldn't be different but it is good to have so we can make sure we don't run into the hard stops.
    private static final double MAX_ANGLE_COMPETITION = Math.toRadians(109.47);
    private static final double MAX_ANGLE_PRACTICE = Math.toRadians(110.41971277880431);

    private static final double ENCODER_GEAR_RATIO = 24.0 / 54.0;
    private static final double ALLOWABLE_TARGET_ANGLE_ERROR = Math.toRadians(4.0); // Allowable error range of 2 degrees

    private static final double ANGLE_FEEDFORWARD = 0.03;

    private static final CargoArmSubsystem instance = new CargoArmSubsystem();

    private final Spark[] motors = {
            new Spark(RobotMap.ARM_MOTOR_A),
            new Spark(RobotMap.ARM_MOTOR_B)
    };

    private final double angleOffset;
    private final double maxAngle;

    private final PidConstants positionPidConstants = new PidConstants(2.0, 1.25, 0.0);
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
    private boolean pitchOnlyDown = false;

    private double previousTimestamp = 0;

    private CargoArmSubsystem() {
        if (Superstructure.getInstance().isPracticeBot()) {
            angleOffset = ANGLE_OFFSET_PRACTICE;
            maxAngle = MAX_ANGLE_PRACTICE;
        } else {
            angleOffset = ANGLE_OFFSET_COMPETITION;
            maxAngle = MAX_ANGLE_COMPETITION;
        }

        positionController.setInputRange(0.0, 2.0 * Math.PI);
        positionController.setContinuous(true);
        positionController.setOutputRange(-1.0, 1.0);
        positionController.setIntegralRange(Math.toRadians(10.0));
    }

    public static CargoArmSubsystem getInstance() {
        return instance;
    }

    public double getMaxAngle() {
        return maxAngle;
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
        angle = MathUtils.clamp(angle, 0.0, maxAngle);

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

    public void setTargetPitch(double pitch, boolean pitchOnlyDown) {
        synchronized (stateLock) {
            targetPitch = pitch;
            this.pitchOnlyDown = pitchOnlyDown;
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
                if (MathUtils.epsilonEquals(currentAngle, 0.0, Math.toRadians(5.0)) && pitchOnlyDown) { // TODO: Make constant
                    output = -0.4; // TODO: Make constant
                }

                if (currentAngle < Math.toRadians(65.0) && currentAngle > Math.toRadians(355.0) && pitchOnlyDown) {
                    output = MathUtils.clamp(output, -1.0, 0.0);
                } else if (!pitchOnlyDown) {
                    // Don't go above 100 degrees
                    if (currentAngle > Math.toRadians(100.0)) {
                        output = 0.0;
                    } else {
                        output = MathUtils.clamp(output, 0.0, 1.0);
                    }
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
        armAngle += angleOffset;
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
