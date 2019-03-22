package org.frcteam2910.c2019.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2019.RobotMap;
import org.frcteam2910.c2019.commands.HolonomicDriveCommand;
import org.frcteam2910.c2019.drivers.Mk2SwerveModule;
import org.frcteam2910.common.control.HolonomicPurePursuitTrajectoryFollower;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.TrajectoryFollower;
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

    private static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-167.1 + 180);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-328.253 + 180);
    private static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-170.8 + 180);
    private static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-3.41);
    private static final double FRONT_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-223.0 + 180.0);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-5.1);
    private static final double BACK_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-357.0 - 180.0);
    private static final double BACK_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-280.0);

    private static final DrivetrainSubsystem instance = new DrivetrainSubsystem();

    private SwerveModule[] swerveModules;

    private HolonomicPurePursuitTrajectoryFollower follower = new HolonomicPurePursuitTrajectoryFollower(12.0, 0.5,
            new HolonomicFeedforward(new DrivetrainFeedforwardConstants(1.0 / (14.0 * 12.0), 0.0, 0.0)),
            new PidConstants(0.0, 0.0, 0.0));

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
    }

    double lastTimestamp = 0;

    final Object lock = new Object();
    HolonomicDriveSignal signal = new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);

    @Override
    public synchronized void updateKinematics(double timestamp) {
        super.updateKinematics(timestamp);

        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        Optional<HolonomicDriveSignal> optSignal = follower.update(new RigidTransform2(getKinematicPosition(),
                getGyroscope().getAngle()), getKinematicVelocity(), getGyroscope().getRate(), timestamp, dt);
        if (optSignal.isPresent()) {
            HolonomicDriveSignal signal = optSignal.get();

            synchronized (lock) {
                this.signal = signal;
            }

            holonomicDrive(signal.getTranslation(), signal.getRotation(), signal.isFieldOriented());
        }
    }

    @Override
    public void outputToSmartDashboard() {
        super.outputToSmartDashboard();

        HolonomicDriveSignal localSignal;
        synchronized (lock) {
            localSignal = signal;
        }

        SmartDashboard.putNumber("Drivetrain Follower Forwards", localSignal.getTranslation().x);
        SmartDashboard.putNumber("Drivetrain Follower Strafe", localSignal.getTranslation().y);
        SmartDashboard.putNumber("Drivetrain Follower Rotation", localSignal.getRotation());
        SmartDashboard.putBoolean("Drivetrain Follower Field Oriented", localSignal.isFieldOriented());
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
