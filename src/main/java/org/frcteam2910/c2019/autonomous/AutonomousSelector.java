package org.frcteam2910.c2019.autonomous;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.frcteam2910.c2019.Robot;
import org.frcteam2910.c2019.commands.DoTheThingCommand;
import org.frcteam2910.c2019.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2019.commands.ImprovedVisionPlaceCommand;
import org.frcteam2910.c2019.commands.SetArmAngleCommand;
import org.frcteam2910.c2019.subsystems.CargoArmSubsystem;
import org.frcteam2910.c2019.subsystems.CargoGrabberSubsystem;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.Side;

import java.util.LinkedList;
import java.util.Queue;

public class AutonomousSelector {
    private final AutonomousTrajectories trajectories;

    private static SendableChooser<Side> sideChooser;
    private static SendableChooser<Rotation2> orientationChooser;
    private static SendableChooser<AutonomousMode> autonomousModeChooser;
    private static NetworkTableEntry onHab2Entry;
    private static NetworkTableEntry placeThirdPanelEntry;
    private static NetworkTableEntry placeFourthPanelEntry;
    private static NetworkTableEntry rocketAutoEntry;

    private Queue<Command> hybridCommandQueue = new LinkedList<>();

    static {
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
        autonomousModeChooser.addOption("Hybrid", AutonomousMode.HYBRID);
        autonomousModeChooser.addOption("Autonomous", AutonomousMode.AUTONOMOUS);
        sandstormTab.add("Mode", autonomousModeChooser);

        onHab2Entry = sandstormTab.add("On HAB 2", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();
        placeThirdPanelEntry = sandstormTab.add("Place 3rd Hatch", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();
        placeFourthPanelEntry = sandstormTab.add("Place 4th Hatch", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();
        rocketAutoEntry = sandstormTab.add("Rocket Auto", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();
    }

    public AutonomousSelector(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;
    }

    private Command getFillRocketBottomCommand() {
        // First leave the hab and drive to the far rocket
        AutonomousMode mode = autonomousModeChooser.getSelected();
        Rotation2 startingOrientation = orientationChooser.getSelected();
        Side startingSide = sideChooser.getSelected();
        boolean onHab2 = onHab2Entry.getBoolean(false);

        CommandGroup group = new CommandGroup();

        if (onHab2) {
            group.addSequential(new FollowTrajectoryCommand(
                    trajectories.getHab2ToRocketFarTrajectory(startingSide)
            ));
        } else {
            group.addSequential(new FollowTrajectoryCommand(
                    trajectories.getHab1ToRocketFarTrajectory(startingSide)
            ));
        }

        group.addSequential(new DoTheThingCommand(true, false));

        // Go back to the loading station and grab the next hatch
        group.addSequential(new FollowTrajectoryCommand(
                trajectories.getRocketFarToLoadingStationTrajectory(startingSide)
        ));
        group.addSequential(new DoTheThingCommand(true, false));

        // Go to the near rocket and place
        group.addSequential(new FollowTrajectoryCommand(
                trajectories.getLoadingStationToRocketNearTrajectory(startingSide)
        ));
        group.addSequential(new DoTheThingCommand(true, false));

        // Put the arm down and start intaking, drive to the depot
        group.addParallel(new SetArmAngleCommand(CargoArmSubsystem.BOTTOM_ANGLE));
        group.addSequential(new FollowTrajectoryCommand(
                trajectories.getRocketNearToDepotTrajectory(startingSide)
        ));

        // Put the arm up and drive to the rocket cargo bay
        group.addParallel(new SetArmAngleCommand(CargoArmSubsystem.ROCKET_SCORE_ANGLE));
        group.addSequential(new FollowTrajectoryCommand(
                trajectories.getDepotToRocketCargoTrajectory(startingSide)
        ));

        return group;
    }

    public Command getCommand() {
        AutonomousMode mode = autonomousModeChooser.getSelected();
        Rotation2 startingOrientation = orientationChooser.getSelected();
        Side startingSide = sideChooser.getSelected();
        boolean onHab2 = onHab2Entry.getBoolean(false);

        CommandGroup group = new CommandGroup();
        group.setRunWhenDisabled(true);

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

        group.addParallel(new SetArmAngleCommand(CargoArmSubsystem.VISION_TARGET_ANGLE));

        if (rocketAutoEntry.getBoolean(false)) {
            group.addSequential(getFillRocketBottomCommand());
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
        hybridCommandQueue.clear();
        // First we want to go to the loading station
        CommandGroup loadingStationPickup1 = new CommandGroup();
        loadingStationPickup1.setRunWhenDisabled(true);
        loadingStationPickup1.addSequential(new FollowTrajectoryCommand(trajectories.getCargoSideNearToLoadingStationTrajectory(startingSide)));
        if (mode == AutonomousMode.AUTONOMOUS) {
            group.addSequential(new DoTheThingCommand(false, false));
            group.addSequential(loadingStationPickup1);
            group.addSequential(new DoTheThingCommand(false, false));
        } else {
            hybridCommandQueue.add(loadingStationPickup1);
        }
        // Next we want to go to the mid cargo ship
        CommandGroup cargoMidPlace = new CommandGroup();
        cargoMidPlace.setRunWhenDisabled(true);
        cargoMidPlace.addSequential(new FollowTrajectoryCommand(trajectories.getLoadingStationToCargoSideMidTrajectory(startingSide)));
        if (mode == AutonomousMode.AUTONOMOUS) {
            group.addSequential(cargoMidPlace);
            group.addSequential(new DoTheThingCommand(false, false));
        } else {
            hybridCommandQueue.add(cargoMidPlace);
        }
        // Finally, drive back to the loading station
        Command loadingStationPickup2 = new FollowTrajectoryCommand(trajectories.getCargoSideMidToLoadingStationTrajectory(startingSide));
        loadingStationPickup2.setRunWhenDisabled(true);
        if (mode == AutonomousMode.AUTONOMOUS) {
            group.addSequential(loadingStationPickup2);
        } else {
            hybridCommandQueue.add(loadingStationPickup2);
        }

        // Place a 3rd hatch panel (Only will work if a hatch can be loaded in sandstorm.)
        if (placeThirdPanelEntry.getBoolean(false)) {
            Command placeThirdPanel = new FollowTrajectoryCommand(trajectories.getLoadingStationToRocketNearTrajectory(startingSide));
            placeThirdPanel.setRunWhenDisabled(true);
            if (mode == AutonomousMode.AUTONOMOUS) {
                group.addSequential(new Command() {
                    @Override
                    protected boolean isFinished() {
                        return Robot.getOi().primaryController.getLeftJoystickButton().get();
                    }
                });
                group.addSequential(new DoTheThingCommand(false, false));
                group.addSequential(placeThirdPanel);
                group.addSequential(new DoTheThingCommand(true, false));
            } else {
                hybridCommandQueue.add(placeThirdPanel);
            }

            // Pickup a 4th hatch because why not
            if (placeFourthPanelEntry.getBoolean(false)) {
                Command pickupFourthPanel = new FollowTrajectoryCommand(trajectories.getRocketNearToLoadingStationTrajectory(startingSide));
                pickupFourthPanel.setRunWhenDisabled(true);
                if (mode == AutonomousMode.AUTONOMOUS) {
                    group.addSequential(pickupFourthPanel);
                    group.addSequential(new DoTheThingCommand(false, false));
                } else {
                    hybridCommandQueue.add(pickupFourthPanel);
                }

                Command placeFourthPanel = new FollowTrajectoryCommand(trajectories.getLoadingStationToRocketFarTrajectory(startingSide));
                placeFourthPanel.setRunWhenDisabled(true);
                if (mode == AutonomousMode.AUTONOMOUS) {
                    group.addSequential(placeFourthPanel);
                    group.addSequential(new DoTheThingCommand(true, true));
                } else {
                    hybridCommandQueue.add(placeFourthPanel);
                }
            }
        }

        return group;
    }

    public Queue<Command> getHybridQueue() {
        return hybridCommandQueue;
    }

    private enum AutonomousMode {
        DRIVEN,
        HYBRID,
        AUTONOMOUS
    }
}
