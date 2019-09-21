package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.Robot;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Vector2;

import java.util.function.Supplier;

public class FollowTrajectoryCommand extends Command {
    private final Supplier<Trajectory> trajectorySupplier;

    private Trajectory trajectory;

    public FollowTrajectoryCommand(Trajectory trajectory) {
        this(() -> trajectory);
    }

    public FollowTrajectoryCommand(Supplier<Trajectory> trajectorySupplier) {
        this.trajectorySupplier = trajectorySupplier;

        requires(DrivetrainSubsystem.getInstance());
        this.setRunWhenDisabled(true);
    }

    @Override
    protected void initialize() {
        trajectory = trajectorySupplier.get();
        DrivetrainSubsystem.getInstance().resetKinematics(Vector2.ZERO, Timer.getFPGATimestamp());
        DrivetrainSubsystem.getInstance().getFollower().follow(trajectory);
    }

    @Override
    protected void end() {
        DrivetrainSubsystem.getInstance().setSnapRotation(trajectory.calculateSegment(trajectory.getDuration()).rotation.toRadians());

        new Thread(() -> {
            Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
            Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
            Timer.delay(0.5);
            Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
            Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        }).start();
    }

    @Override
    protected void interrupted() {
        end();
        DrivetrainSubsystem.getInstance().getFollower().cancel();
    }

    @Override
    protected boolean isFinished() {
        // Only finish when the trajectory is completed
        return DrivetrainSubsystem.getInstance().getFollower().getCurrentTrajectory().isEmpty();
    }
}
