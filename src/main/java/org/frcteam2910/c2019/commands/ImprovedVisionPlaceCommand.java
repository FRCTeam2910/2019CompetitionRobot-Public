package org.frcteam2910.c2019.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2019.Robot;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2019.subsystems.HatchPlacerSubsystem;
import org.frcteam2910.c2019.subsystems.Superstructure;
import org.frcteam2910.c2019.subsystems.VisionSubsystem;
import org.frcteam2910.c2019.vision.api.Gamepiece;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Limelight;
import org.frcteam2910.common.robot.subsystems.Drivetrain;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.MovingAverage;

import java.util.function.BiFunction;

public class ImprovedVisionPlaceCommand extends Command {
    private static final double MAX_EFFORT_DISTANCE = 72.0;
    private static final double MIN_EFFORT_DISTANCE = 24.0;
    private static final double MIN_DISTANCE_EFFORT = 0.3;

    private static final double MAX_EFFORT_ANGLE_ERROR = Math.toRadians(0.0);
    private static final double MIN_EFFORT_ANGLE_ERROR = Math.toRadians(20.0);

    private static final double MAX_EFFORT_HORIZ_ERROR = 0.0;
    private static final double MIN_EFFORT_HORIZ_ERROR = 36.0;

    private static final double CLOSE_CORRECTION_MAX_EFFORT_HORIZ_ERROR = 0.75;
    private static final double CLOSE_CORRECTION_ZERO_EFFORT_HORIZ_ERROR = 1.0;
    private static final double CLOSE_CORRECTION_MAX_REVERSE_EFFORT_HORIZ_ERROR = 5.0;

    private static final double PLACEMENT_DISTANCE = 27.0;
    private static final double CLOSE_CORRECTION_DISTANCE_FROM_PLACEMENT = 10.0;

    private static final Rotation2[] VALID_TARGET_ANGLES = {
            Rotation2.ZERO, // Loading Station
            Rotation2.fromDegrees(90), // Left Cargo Ship
            Rotation2.fromDegrees(180), // Front Cargo Ship
            Rotation2.fromDegrees(270), // Right Cargo Ship
            Rotation2.fromDegrees(212), // Front Left Rocket
            Rotation2.fromDegrees(148), // Front Right Rocket
            Rotation2.fromDegrees(328), // Back Left Rocket
            Rotation2.fromDegrees(32), // Back Right Rocket
    };

    private static final double CARGO_CLEAR_THRESHOLD = 0.75;

    private static final double LOCK_ON_TARGET_ALLOWABLE_ERROR = 10.0;
    private static final double LOCK_ON_TARGET_TRIES = 5;

    private static final int FINISH_SUCCESSES_NEEDED = 4;
    private static final double TARGET_ANGLE_ACCEPTABLE_ERROR = Math.toRadians(5.0);

    private static final int HIGH_RESOLUTION_HEIGHT = 720;
    private static final int LOW_RESOLUTION_HEIGHT = 240;

    private static final int HIGH_RESOLUTION_PIPELINE = 9;
    private static final int LOW_RESOLUTION_PIPELINE = 8;

    private static NetworkTableEntry shouldSnapToAngleEntry;
    private static NetworkTableEntry pickupFromAnyTargetEntry;

    private final PidController controller = new PidController(new PidConstants(0.015, 0.0, 0.1));

    private Limelight limelight = VisionSubsystem.getInstance().getLimelight(Gamepiece.HATCH_PANEL);
    private NetworkTable limelightTable = limelight.getTable();
    private NetworkTableEntry targetTransformEntry = limelightTable.getEntry("camtran");

    private final double maxForwardSpeed;
    private final BiFunction<Double, Double, Boolean> finishCondition;
    private final double distanceToStopRotation;
    private final boolean useCloseCorrection;

    private double lastTime;
    private Rotation2 targetAngle;
    private double lockOnTargetTriesLeft;
    private boolean foundTarget;
    private Vector2 fieldOrientedPositionOffset;
    private Vector2 robotOrientedPositionOffset;
    private double distance = Double.POSITIVE_INFINITY;

    private MovingAverage averageRobotYOffset = new MovingAverage(3);

    private int successesNeeded;

    static {
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        shouldSnapToAngleEntry = tab.add("Snap to Angle", false)
                .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        pickupFromAnyTargetEntry = tab.add("Pickup From Any Target", false)
                .withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }

    public ImprovedVisionPlaceCommand(double maxForwardSpeed, BiFunction<Double, Double, Boolean> finishCondition) {
        this(maxForwardSpeed, finishCondition, 0.0, true);
    }

    public ImprovedVisionPlaceCommand(double maxForwardSpeed, BiFunction<Double, Double, Boolean> finishCondition,
                                      double distanceToStopRotation, boolean useCloseCorrection) {
        this.maxForwardSpeed = maxForwardSpeed;
        this.finishCondition = finishCondition;
        this.distanceToStopRotation = distanceToStopRotation;
        this.useCloseCorrection = useCloseCorrection;

        controller.setOutputRange(-0.5, 0.5);
        controller.setIntegralRange(12.0);
        controller.setShouldClearIntegralOnErrorSignChange(true);

        requires(DrivetrainSubsystem.getInstance());

        this.setRunWhenDisabled(true);
    }

    @Override
    protected void initialize() {
        lastTime = Timer.getFPGATimestamp();

        controller.reset();

        limelight.setCamMode(Limelight.CamMode.VISION);
        if (shouldSnapToAngleEntry.getBoolean(false)) {
            limelight.setPipeline(HIGH_RESOLUTION_PIPELINE);
        } else {
            limelight.setPipeline(LOW_RESOLUTION_PIPELINE);
        }
        targetAngle = null;
        foundTarget = false;
        lockOnTargetTriesLeft = LOCK_ON_TARGET_TRIES;
        fieldOrientedPositionOffset = null;
        distance = Double.POSITIVE_INFINITY;
        averageRobotYOffset.clear();
        successesNeeded = FINISH_SUCCESSES_NEEDED;

        // If we don't have a hatch panel we know the target is the loading station which has an angle of
        if (!pickupFromAnyTargetEntry.getBoolean(false)) {
            if (!HatchPlacerSubsystem.getInstance().hasHatch()) {
                targetAngle = Rotation2.ZERO;
                foundTarget = true;
            }
        }
    }

    @Override
    protected void execute() {
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;

        if (foundTarget) {
            Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
        } else {
            Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        }

        double[] targetTransform = targetTransformEntry.getDoubleArray((double[]) null);
        if (Math.abs(Robot.getOi().primaryController.getRightXAxis().get(false, true)) > CARGO_CLEAR_THRESHOLD) {
            DrivetrainSubsystem.getInstance().holonomicDrive(
                    new Vector2(
                            Robot.getOi().primaryController.getLeftYAxis().get(true),
                            Robot.getOi().primaryController.getLeftXAxis().get(true)
                    ),
                    Math.copySign(1.0, Robot.getOi().primaryController.getRightXAxis().get()),
                    true
            );
        } else if (targetTransform == null || !limelight.hasTarget()) {
            // We can't see a target

            if (foundTarget && fieldOrientedPositionOffset != null) {
                // We know where the target should be
                DrivetrainSubsystem.getInstance().setSnapRotation(targetAngle.toRadians());
                DrivetrainSubsystem.getInstance().setTargetPose(
                        new RigidTransform2(
                                DrivetrainSubsystem.getInstance().getKinematicPosition().add(fieldOrientedPositionOffset),
                                targetAngle
                        ),
                        0.0
                );
            } else if (foundTarget) {
                // We know the angle of the target but not where it should be
                // Take control of the rotation but let the driver control translation
                DrivetrainSubsystem.getInstance().setSnapRotation(targetAngle.toRadians());
                DrivetrainSubsystem.getInstance().holonomicDrive(
                        new Vector2(
                                Robot.getOi().primaryController.getLeftYAxis().get(true),
                                Robot.getOi().primaryController.getLeftXAxis().get(true)
                        ),
                        0.0,
                        true
                );
            } else {
                // We don't know where the target should be
                DrivetrainSubsystem.getInstance().holonomicDrive(
                        new Vector2(
                                Robot.getOi().primaryController.getLeftYAxis().get(true),
                                Robot.getOi().primaryController.getLeftXAxis().get(true)
                        ),
                        Robot.getOi().primaryController.getRightXAxis().get(true),
                        true
                );
            }
        } else {
            // We can see a target

            {
                double theta;
                if (shouldSnapToAngleEntry.getBoolean(false)) {
                    theta = limelight.getTable().getEntry("tvert").getDouble(0.0) * Math.toRadians(49.7) / HIGH_RESOLUTION_HEIGHT;
                } else {
                    theta = limelight.getTable().getEntry("tvert").getDouble(0.0) * Math.toRadians(49.7) / LOW_RESOLUTION_HEIGHT;
                }
                distance = 5.83 / (2 * Math.tan(theta / 2));

                robotOrientedPositionOffset = Vector2.fromAngle(Rotation2.fromRadians(limelight.getTargetPosition().x)).scale(distance);
                double robotOrientedYOffset = distance * Math.sin(limelight.getTargetPosition().x);

                fieldOrientedPositionOffset = Vector2.fromAngle(
                        Superstructure.getInstance().getGyroscope().getAngle()
                                .rotateBy(Rotation2.fromDegrees(90))
                ).scale(robotOrientedYOffset);
            }
            averageRobotYOffset.add(robotOrientedPositionOffset.y);

            if (!foundTarget && shouldSnapToAngleEntry.getBoolean(true)) {
                Rotation2 targetYawOffset = Rotation2.fromDegrees(targetTransform[4]);
                Rotation2 fieldOrientedTargetRotation = Superstructure.getInstance().getGyroscope().getAngle()
                        .rotateBy(targetYawOffset.inverse());

                Rotation2 bestTarget = null;

                for (Rotation2 validTargetAngle : VALID_TARGET_ANGLES) {
                    Rotation2 delta = Vector2.getAngleBetween(
                            Vector2.fromAngle(fieldOrientedTargetRotation),
                            Vector2.fromAngle(validTargetAngle)
                    );

                    if (delta.toDegrees() < LOCK_ON_TARGET_ALLOWABLE_ERROR) {
                        bestTarget = validTargetAngle;
                        break;
                    }
                }

                if (bestTarget == null) {
                    lockOnTargetTriesLeft = LOCK_ON_TARGET_TRIES;
                } else {
                    if (bestTarget == targetAngle) {
                        lockOnTargetTriesLeft--;
                    } else {
                        lockOnTargetTriesLeft = LOCK_ON_TARGET_TRIES;
                    }

                    if (lockOnTargetTriesLeft == 0) {
                        foundTarget = true;
                    }

                    targetAngle = bestTarget;
                }

                DrivetrainSubsystem.getInstance().holonomicDrive(
                        new Vector2(
                                Robot.getOi().primaryController.getLeftYAxis().get(true),
                                Robot.getOi().primaryController.getLeftXAxis().get(true)
                        ),
                        Robot.getOi().primaryController.getRightXAxis().get(true),
                        true
                );
            } else {
                Rotation2 targetAngleError = Rotation2.ZERO;
                if (targetAngle != null) {
                     targetAngleError = Vector2.getAngleBetween(
                            Vector2.fromAngle(targetAngle),
                            Vector2.fromAngle(Superstructure.getInstance().getGyroscope().getAngle())
                    );
                }

                double forwardAngleEffort = MathUtils.lerp(0.0, 1.0,
                        MathUtils.inverseLerp(MIN_EFFORT_ANGLE_ERROR, MAX_EFFORT_ANGLE_ERROR, targetAngleError.toRadians()));

                double forwardDistanceEffort = MathUtils.lerp(MIN_DISTANCE_EFFORT, 1.0,
                        MathUtils.inverseLerp(MIN_EFFORT_DISTANCE, MAX_EFFORT_DISTANCE, distance));

                double forwardHorizEffort = MathUtils.lerp(0.0, 1.0,
                        MathUtils.inverseLerp(MIN_EFFORT_HORIZ_ERROR, MAX_EFFORT_HORIZ_ERROR, fieldOrientedPositionOffset.length));

                double closeCorrectionEffort = 1.0;
                if (useCloseCorrection) {
                    double a = MathUtils.lerp(0.0, 1.0,
                            MathUtils.inverseLerp(CLOSE_CORRECTION_ZERO_EFFORT_HORIZ_ERROR, CLOSE_CORRECTION_MAX_EFFORT_HORIZ_ERROR, fieldOrientedPositionOffset.length));

                    double b = MathUtils.lerp(0.0, -1.0,
                            MathUtils.inverseLerp(CLOSE_CORRECTION_ZERO_EFFORT_HORIZ_ERROR, CLOSE_CORRECTION_MAX_REVERSE_EFFORT_HORIZ_ERROR, fieldOrientedPositionOffset.length));

                    closeCorrectionEffort = MathUtils.lerp(a + b, 1.0,
                            MathUtils.inverseLerp(PLACEMENT_DISTANCE, PLACEMENT_DISTANCE + CLOSE_CORRECTION_DISTANCE_FROM_PLACEMENT, distance));
                }

                SmartDashboard.putNumber("Horizontal Error", robotOrientedPositionOffset.y);
                SmartDashboard.putNumber("Angle Effort", forwardAngleEffort);
                SmartDashboard.putNumber("Distance Effort", forwardDistanceEffort);
                SmartDashboard.putNumber("Horiz Effort", forwardHorizEffort);
                SmartDashboard.putNumber("Close Correction Effort", closeCorrectionEffort);

                double forwardPercentOutput = maxForwardSpeed * forwardAngleEffort * forwardDistanceEffort * forwardHorizEffort * closeCorrectionEffort;

                SmartDashboard.putNumber("Total Forward Effort", forwardPercentOutput);

                if (distance < distanceToStopRotation && targetAngleError.toRadians() < TARGET_ANGLE_ACCEPTABLE_ERROR) {
                    DrivetrainSubsystem.getInstance().stopSnap();
                } else if (targetAngle != null) {
                    DrivetrainSubsystem.getInstance().setSnapRotation(targetAngle.toRadians());
                }

                double strafePercentOutput = -controller.calculate(averageRobotYOffset.get(), dt);
                strafePercentOutput = Math.copySign(Math.max(Math.abs(strafePercentOutput), 0.025), strafePercentOutput);

                double rotationPercentOutput = 0.0;
                if (targetAngle == null) {
                    rotationPercentOutput = Robot.getOi().primaryController.getRightXAxis().get() * 0.5;
                }

                DrivetrainSubsystem.getInstance().holonomicDrive(
                        new Vector2(
                                -forwardPercentOutput,
                                strafePercentOutput
                        ),
                        rotationPercentOutput,
                        false
                );
            }
        }
    }

    @Override
    protected boolean isFinished() {
        if (robotOrientedPositionOffset == null) {
            return false;
        }

        if (finishCondition.apply(distance, robotOrientedPositionOffset.y)) {
            successesNeeded--;
        } else {
            successesNeeded = FINISH_SUCCESSES_NEEDED;
        }

        return successesNeeded == 0;
    }

    @Override
    protected void end() {
        DrivetrainSubsystem.getInstance().stop();
        Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        limelight.setCamMode(Limelight.CamMode.DRIVER);
    }
}
