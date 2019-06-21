package org.frcteam2910.c2019.autonomous;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.frcteam2910.c2019.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.util.Side;

import java.util.LinkedList;
import java.util.Queue;

public class AutonomousSelector {
    private final AutonomousTrajectories trajectories;

    private SendableChooser<Side> sideChooser;
    private SendableChooser<Rotation2> orientationChooser;
    private SendableChooser<AutonomousMode> autonomousModeChooser;
    private NetworkTableEntry onHab2Entry;

    private Queue<Trajectory> hybridTrajectoryQueue = new LinkedList<>();

    public AutonomousSelector(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        ShuffleboardTab sandstormTab = Shuffleboard.getTab("Sandstorm settings");

        sideChooser = new SendableChooser<>();
        sideChooser.addOption("Left", Side.LEFT);
        sideChooser.setDefaultOption("Right", Side.RIGHT);
        sandstormTab.add("Starting Side", sideChooser);

        orientationChooser = new SendableChooser<>();
        orientationChooser.setDefaultOption("Forward", Rotation2.ZERO);
        orientationChooser.addOption("Backwards", Rotation2.fromDegrees(180.0));
        orientationChooser.addOption("Left", Rotation2.fromDegrees(90.0));
        orientationChooser.addOption("Right", Rotation2.fromDegrees(270.0));
        sandstormTab.add("Starting Orientation", orientationChooser);

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.setDefaultOption("Driven", AutonomousMode.DRIVEN);
        autonomousModeChooser.addOption("Autonomous", AutonomousMode.AUTONOMOUS);
        sandstormTab.add("Mode", autonomousModeChooser);

        onHab2Entry = sandstormTab.add("On HAB 2", false).getEntry();
    }

    public Command getCommand() {
        AutonomousMode mode = autonomousModeChooser.getSelected();
        Rotation2 startingOrientation = orientationChooser.getSelected();
        Side startingSide = sideChooser.getSelected();
        boolean onHab2 = onHab2Entry.getBoolean(false);

        CommandGroup group = new CommandGroup();

        // Set the gyro angle to the correct starting angle
        group.addSequential(new InstantCommand(() -> {
            DrivetrainSubsystem.getInstance().getGyroscope().setAdjustmentAngle(
                    DrivetrainSubsystem.getInstance().getGyroscope().getUnadjustedAngle().rotateBy(startingOrientation)
            );
        }));

        // If we want to manually drive the robot, return now.
        if (mode == AutonomousMode.DRIVEN) {
            return group;
        }

        // Drive to the first target
        // If we are on hab 2, leave hab 2 in a (semi) repeatable manner
        if (onHab2) {
            group.addSequential(new FollowTrajectoryCommand(trajectories.getHab2ToCargoSideNearTrajectory(startingSide)));
        } else {
            group.addSequential(new FollowTrajectoryCommand(trajectories.getHab1ToCargoSideNearTrajectory(startingSide)));
        }
        // Enqueue the next trajectories
        hybridTrajectoryQueue.clear();
        // First we want to go to the loading station
        hybridTrajectoryQueue.add(trajectories.getCargoSideNearToLoadingStationTrajectory(startingSide));
        // Next we want to go to the mid cargo ship
        hybridTrajectoryQueue.add(trajectories.getLoadingStationToCargoSideMid(startingSide));
        // Finally, drive back to the loading station
        hybridTrajectoryQueue.add(trajectories.getCargoSideMidToLoadingStationTrajectory(startingSide));

        return group;
    }

    public Queue<Trajectory> getTrajectoryQueue() {
        return hybridTrajectoryQueue;
    }

    private enum AutonomousMode {
        DRIVEN,
        AUTONOMOUS
    }
}
