package org.frcteam2910.c2019;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.frcteam2910.c2019.autonomous.AutonomousSelector;
import org.frcteam2910.c2019.commands.*;
import org.frcteam2910.c2019.subsystems.CargoArmSubsystem;
import org.frcteam2910.c2019.subsystems.CargoGrabberSubsystem;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2019.subsystems.HatchFloorGathererSubsystem;
import org.frcteam2910.c2019.vision.api.Gamepiece;
import org.frcteam2910.common.robot.commands.ZeroFieldOrientedCommand;
import org.frcteam2910.common.robot.input.DPadButton;
import org.frcteam2910.common.robot.input.XboxController;

public class OI {
    public final XboxController primaryController = new XboxController(0);

    public final XboxController secondaryController = new XboxController(1);

    public OI() {
        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        primaryController.getRightXAxis().setScale(0.75);
    }

    public void bindButtons(AutonomousSelector autonomousSelector) {
        primaryController.getRightTriggerAxis().whenPressed(new ExtendHatchPlacerCommand());
        {
            CommandGroup group = new CommandGroup();
            group.addSequential(new ReleaseHatchPanelCommand());
            group.addSequential(new WaitCommand(0.1));
            group.addSequential(new RetractHatchPlacerCommand());
            primaryController.getRightTriggerAxis().whenReleased(group);
        }

        primaryController.getAButton().whenPressed(new SetClimberExtendedCommand(true));
        primaryController.getAButton().whenReleased(new SetClimberExtendedCommand(false));

        primaryController.getRightBumperButton().whenPressed(new SetArmAngleCommand(CargoArmSubsystem.VISION_TARGET_ANGLE));
        primaryController.getRightBumperButton().whileHeld(new VisionTargetCommand(Gamepiece.HATCH_PANEL));
        primaryController.getRightBumperButton().whenReleased(new ConditionalCommand(
                new FollowTrajectoryCommand(autonomousSelector.getTrajectoryQueue()::remove)
        ) {
            @Override
            protected boolean condition() {
                System.out.printf("Checking %d%n", autonomousSelector.getTrajectoryQueue().size());
                return DriverStation.getInstance().isAutonomous() && !autonomousSelector.getTrajectoryQueue().isEmpty();
            }
        });

        primaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(
                new SetHatchFloorGathererAngleCommand(HatchFloorGathererSubsystem.getInstance().getMaxAngle()));
        primaryController.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(new SetHatchFloorGathererAngleCommand(Math.toRadians(90.0)));
        primaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(new SetHatchFloorGathererAngleCommand(0.0));
        primaryController.getDPadButton(DPadButton.Direction.LEFT).whenPressed(new RetractKickstandCommand());

        // Field oriented zero
        primaryController.getBackButton().whenPressed(new ZeroFieldOrientedCommand(DrivetrainSubsystem.getInstance()));

        // Climbing
        primaryController.getStartButton().whenPressed(new ConditionalCommand(new OverhangCommand(), new ClimbCommand()) {
            private SendableChooser<Boolean> climbModeSendable = new SendableChooser<>();

            {
                ShuffleboardTab climbTab = Shuffleboard.getTab("Climbing");

                climbModeSendable.setName("Climb Mode");
                climbModeSendable.setDefaultOption("Normal", false);
                climbModeSendable.addOption("Overhang", true);
                climbTab.add(climbModeSendable);
            }

            @Override
            protected boolean condition() {
                return climbModeSendable.getSelected();
            }
        });
        primaryController.getStartButton().whenReleased(new AbortClimbCommand());


        // Cargo arm top position
        secondaryController.getDPadButton(DPadButton.Direction.UP).whileHeld(new SetArmAngleCommand(
                CargoArmSubsystem.getInstance().getMaxAngle()));
        // Cargo arm cargo ship height
        secondaryController.getDPadButton(DPadButton.Direction.LEFT).whenPressed(
                new SetArmAngleCommand(CargoArmSubsystem.CARGO_SHIP_SCORE_ANGLE));
        // Cargo arm rocket & climb height
        secondaryController.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(
                new SetArmAngleCommand(CargoArmSubsystem.ROCKET_SCORE_ANGLE));
        // Cargo arm bottom position
        secondaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(new SetArmAngleCommand(CargoArmSubsystem.BOTTOM_ANGLE));

        {
            CommandGroup group = new CommandGroup();
            group.addSequential(new ReleaseHatchPanelCommand());
            group.addSequential(new ExtendHatchPlacerCommand());
            secondaryController.getRightTriggerAxis().whenPressed(group);

        }

        {
            CommandGroup group = new CommandGroup();
            group.addParallel(new GrabOnHasHatchPanelCommand());
            group.addSequential(new RumbleWhileHasHatchPanelCommand());
            secondaryController.getRightTriggerAxis().whileHeld(group);

        }
        {
            CommandGroup group = new CommandGroup();
            group.addSequential(new GrabHatchPanelCommand());
            group.addSequential(new WaitCommand(0.1));
            group.addSequential(new RetractHatchPlacerCommand());
            secondaryController.getRightTriggerAxis().whenReleased(group);
        }

        secondaryController.getRightBumperButton().whileHeld(new GrabHatchFromFloorPart1Command());
        secondaryController.getRightBumperButton().whenReleased(new GrabHatchFromFloorPart2Command());

        secondaryController.getLeftTriggerAxis().whileHeld(new Command() {
            @Override
            protected void execute() {
                if (CargoGrabberSubsystem.getInstance().hasLeftCargo() || CargoGrabberSubsystem.getInstance().hasRightCargo()) {
                    secondaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
                } else {
                    secondaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                }
            }

            @Override
            protected void end() {
                secondaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        });

        secondaryController.getAButton().whenPressed(new SetHatchFloorGathererAngleCommand(
                HatchFloorGathererSubsystem.getInstance().getMaxAngle()));
    }
}

