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

    private static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-50.00 + 180.0);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-284.47 + 180.0);
    private static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-331.52 + 180.0);
    private static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-209.22);
    private static final double FRONT_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-56.53 + 180);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-109.38 + 180);
    private static final double BACK_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-4.21 + 180);
    private static final double BACK_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-332.17);

    private static final PidConstants FOLLOWER_TRANSLATION_CONSTANTS = new PidConstants(0.05, 0.01, 0.0);
    private static final PidConstants FOLLOWER_ROTATION_CONSTANTS = new PidConstants(0.2, 0.01, 0.0);
    private static final HolonomicFeedforward FOLLOWER_FEEDFORWARD_CONSTANTS = new HolonomicFeedforward(
            new DrivetrainFeedforwardConstants(1.0 / (14.0 * 12.0), 0.0, 0.0)
    );

    private static final PidConstants SNAP_ROTATION_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);

    private static final DrivetrainSubsystem instance = new DrivetrainSubsystem();

    private SwerveModule[] swerveModules;

    private HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            FOLLOWER_TRANSLATION_CONSTANTS,
            FOLLOWER_ROTATION_CONSTANTS,
            FOLLOWER_FEEDFORWARD_CONSTANTS
    );

    private PidController snapRotationController = new PidController(SNAP_ROTATION_CONSTANTS);
    private double snapRotation = Double.NaN;

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

        SwerveModule frontLeftModule = new Mk2SwerveModule(
                new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                frontLeftAngleOffset,
                new Spark(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER)
        );
        frontLeftModule.setName("Front Left");

        SwerveModule frontRightModule = new Mk2SwerveModule(
                new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                frontRightAngleOffset,
                new Spark(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER)
        );
        frontRightModule.setName("Front Right");

        SwerveModule backLeftModule = new Mk2SwerveModule(
                new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                backLeftAngleOffset,
                new Spark(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER)
        );
        backLeftModule.setName("Back Left");

        SwerveModule backRightModule = new Mk2SwerveModule(
                new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                backRightAngleOffset,
                new Spark(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER)
        );
        backRightModule.setName("Back Right");

        swerveModules = new SwerveModule[]{
                frontLeftModule,
                frontRightModule,
                backLeftModule,
                backRightModule,
        };

        snapRotationController.setInputRange(0.0, 2.0 * Math.PI);
        snapRotationController.setContinuous(true);
    }

    public void setSnapRotation(double snapRotation) {
        synchronized (lock) {
            this.snapRotation = snapRotation;
        }
    }

    @Override
    public void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented) {
        synchronized (lock) {
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

        Optional<HolonomicDriveSignal> optSignal = follower.update(new RigidTransform2(getKinematicPosition(),
                getGyroscope().getAngle()), getKinematicVelocity(), getGyroscope().getRate(), timestamp, dt);
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

    public TrajectoryFollower<HolonomicDriveSignal> getFollower() {
        return follower;
    }
}
