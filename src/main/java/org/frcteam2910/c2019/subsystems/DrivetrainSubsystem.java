package org.frcteam2910.c2019.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2019.RobotMap;
import org.frcteam2910.c2019.commands.HolonomicDriveCommand;
import org.frcteam2910.c2019.drivers.Mk2SwerveModule;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.subsystems.SwerveDrivetrain;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;

import java.util.Optional;

public class DrivetrainSubsystem extends SwerveDrivetrain {
    private static final double TRACKWIDTH = 19.5;
    private static final double WHEELBASE = 23.5;

    public static final ITrajectoryConstraint[] CONSTRAINTS = {
            new MaxVelocityConstraint(12.0 * 12.0),
            new MaxAccelerationConstraint(15.0 * 12.0),
            new CentripetalAccelerationConstraint(25.0 * 12.0)
    };

    private static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-176.3204627908379 + 180.0);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-222.03175099310178);
    private static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-90.48893143542519 + 180.0);
    private static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-268.64311864548347);
    private static final double FRONT_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-346.33384369870345 + 180.0);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-47.735857257929055);
    private static final double BACK_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-57.1115292960996 + 180.0);
    private static final double BACK_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-122.9922320247071);

    private static final PidConstants FOLLOWER_TRANSLATION_CONSTANTS = new PidConstants(0.05, 0.01, 0.0);
    private static final PidConstants FOLLOWER_ROTATION_CONSTANTS = new PidConstants(0.2, 0.01, 0.0);
    private static final HolonomicFeedforward FOLLOWER_FEEDFORWARD_CONSTANTS = new HolonomicFeedforward(
            new DrivetrainFeedforwardConstants(1.0 / (14.0 * 12.0), 0.0, 0.0)
    );

    private static final PidConstants SNAP_ROTATION_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);
    private static final PidConstants SNAP_TRANSLATION_CONSTANTS = new PidConstants(0.02, 0.0, 0.001);

    private static final DrivetrainSubsystem instance = new DrivetrainSubsystem();

    private Mk2SwerveModule[] swerveModules;

    private HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            FOLLOWER_TRANSLATION_CONSTANTS,
            FOLLOWER_ROTATION_CONSTANTS,
            FOLLOWER_FEEDFORWARD_CONSTANTS
    );

    private PidController snapRotationController = new PidController(SNAP_ROTATION_CONSTANTS);
    private double snapRotation = Double.NaN;

    private PidController xPosController = new PidController(SNAP_TRANSLATION_CONSTANTS);
    private PidController yPosController = new PidController(SNAP_TRANSLATION_CONSTANTS);
    private RigidTransform2 targetPose = null;
    private double targetForwardPercentOutput;

    private double lastTimestamp = 0;

    private final Object lock = new Object();
    private HolonomicDriveSignal signal = new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);
    private Trajectory.Segment segment = null;

    private DrivetrainSubsystem() {
        double frontLeftAngleOffset = FRONT_LEFT_ANGLE_OFFSET_COMPETITION;
        double frontRightAngleOffset = FRONT_RIGHT_ANGLE_OFFSET_COMPETITION;
        double backLeftAngleOffset = BACK_LEFT_ANGLE_OFFSET_COMPETITION;
        double backRightAngleOffset = BACK_RIGHT_ANGLE_OFFSET_COMPETITION;
        if (Superstructure.getInstance().isPracticeBot()) {
            frontLeftAngleOffset = FRONT_LEFT_ANGLE_OFFSET_PRACTICE;
            frontRightAngleOffset = FRONT_RIGHT_ANGLE_OFFSET_PRACTICE;
            backLeftAngleOffset = BACK_LEFT_ANGLE_OFFSET_PRACTICE;
            backRightAngleOffset = BACK_RIGHT_ANGLE_OFFSET_PRACTICE;
        }

        Mk2SwerveModule frontLeftModule = new Mk2SwerveModule(
                new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                frontLeftAngleOffset,
                new Spark(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER)
        );
        frontLeftModule.setName("Front Left");

        Mk2SwerveModule frontRightModule = new Mk2SwerveModule(
                new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                frontRightAngleOffset,
                new Spark(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER)
        );
        frontRightModule.setName("Front Right");

        Mk2SwerveModule backLeftModule = new Mk2SwerveModule(
                new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                backLeftAngleOffset,
                new Spark(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER)
        );
        backLeftModule.setName("Back Left");

        Mk2SwerveModule backRightModule = new Mk2SwerveModule(
                new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                backRightAngleOffset,
                new Spark(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER)
        );
        backRightModule.setName("Back Right");

        swerveModules = new Mk2SwerveModule[]{
                frontLeftModule,
                frontRightModule,
                backLeftModule,
                backRightModule,
        };

        snapRotationController.setInputRange(0.0, 2.0 * Math.PI);
        snapRotationController.setContinuous(true);
        snapRotationController.setOutputRange(-0.5, 0.5);

//        xPosController.setOutputRange(-0.15, 0.15);
//        yPosController.setOutputRange(-0.15, 0.15);
    }

    public void setSnapRotation(double snapRotation) {
        synchronized (lock) {
            this.snapRotation = snapRotation;
        }
    }

    public void stopSnap() {
        synchronized (lock) {
            this.snapRotation = Double.NaN;
        }
    }

    public void setTargetPose(RigidTransform2 pose, double forwardPercentOutput) {
        synchronized (lock) {
            targetPose = pose;
            targetForwardPercentOutput = forwardPercentOutput;
        }
    }

    @Override
    public void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented) {
        synchronized (lock) {
            targetPose = null;
            this.signal = new HolonomicDriveSignal(translation, rotation, fieldOriented);
        }
    }

    @Override
    public synchronized void updateKinematics(double timestamp) {
        super.updateKinematics(timestamp);

        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        double localSnapRotation;
        synchronized (lock) {
            localSnapRotation = snapRotation;
        }
        RigidTransform2 currentPose = new RigidTransform2(
                getKinematicPosition(),
                getGyroscope().getAngle()
        );

        Optional<HolonomicDriveSignal> optSignal = follower.update(currentPose, getKinematicVelocity(),
                getGyroscope().getRate(), timestamp, dt);
        HolonomicDriveSignal localSignal;

        if (optSignal.isPresent()) {
            localSignal = optSignal.get();

            synchronized (lock) {
                signal = localSignal;
                segment = follower.getLastSegment();
            }
        } else {
            synchronized (lock) {
                localSignal = this.signal;
            }
        }

        if (Math.abs(localSignal.getRotation()) < 0.1 && Double.isFinite(localSnapRotation)) {
            snapRotationController.setSetpoint(localSnapRotation);

            localSignal = new HolonomicDriveSignal(localSignal.getTranslation(),
                    snapRotationController.calculate(getGyroscope().getAngle().toRadians(), dt),
                    localSignal.isFieldOriented());
        } else {
            synchronized (lock) {
                snapRotation = Double.NaN;
            }
        }

        // Target pose overrides everything
        RigidTransform2 localTargetPose;
        double localTargetForwardPercentOutput;
        synchronized (lock) {
            localTargetPose = targetPose;
            localTargetForwardPercentOutput = targetForwardPercentOutput;
        }
        if (localTargetPose != null) {
            xPosController.setSetpoint(localTargetPose.translation.x);
            yPosController.setSetpoint(localTargetPose.translation.y);
            snapRotationController.setSetpoint(localTargetPose.rotation.toRadians());

            localSignal = new HolonomicDriveSignal(
                    new Vector2(
                            xPosController.calculate(currentPose.translation.x, dt),
                            yPosController.calculate(currentPose.translation.y, dt)
                    ).rotateBy(currentPose.rotation.inverse())
                    .multiply(0.0, 1.0)
                    .add(localTargetForwardPercentOutput, 0.0),
                    snapRotationController.calculate(currentPose.rotation.toRadians(), dt),
                    false
            );
        } else {
            xPosController.reset();
            yPosController.reset();
        }

        super.holonomicDrive(localSignal.getTranslation(), localSignal.getRotation(), localSignal.isFieldOriented());
    }

    @Override
    public void outputToSmartDashboard() {
        super.outputToSmartDashboard();

        HolonomicDriveSignal localSignal;
        Trajectory.Segment localSegment;
        synchronized (lock) {
            localSignal = signal;
            localSegment = segment;
        }

        SmartDashboard.putNumber("Drivetrain Follower Forwards", localSignal.getTranslation().x);
        SmartDashboard.putNumber("Drivetrain Follower Strafe", localSignal.getTranslation().y);
        SmartDashboard.putNumber("Drivetrain Follower Rotation", localSignal.getRotation());
        SmartDashboard.putBoolean("Drivetrain Follower Field Oriented", localSignal.isFieldOriented());

        if (follower.getCurrentTrajectory().isPresent() && localSegment != null) {
            SmartDashboard.putNumber("Drivetrain Follower Target Angle", localSegment.rotation.toDegrees());

            Vector2 position = getKinematicPosition();

            SmartDashboard.putNumber("Drivetrain Follower X Error", localSegment.translation.x - position.x);
            SmartDashboard.putNumber("Drivetrain Follower Y Error", localSegment.translation.y - position.y);
            SmartDashboard.putNumber("Drivetrain Follower Angle Error", localSegment.rotation.toDegrees() - getGyroscope().getAngle().toDegrees());
        }

        for (Mk2SwerveModule module : swerveModules) {
            SmartDashboard.putNumber(String.format("%s Module Drive Current Draw", module.getName()), module.getDriveCurrent());
        }
    }

    public static DrivetrainSubsystem getInstance() {
        return instance;
    }

    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    @Override
    public Gyroscope getGyroscope() {
        return Superstructure.getInstance().getGyroscope();
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
        setDefaultCommand(new HolonomicDriveCommand());
    }

    @Override
    public void stop() {
        super.stop();
        synchronized (lock) {
            snapRotation = Double.NaN;
            targetPose = null;
        }
    }

    public TrajectoryFollower<HolonomicDriveSignal> getFollower() {
        return follower;
    }
}
