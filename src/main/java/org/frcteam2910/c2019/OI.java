package org.frcteam2910.c2019;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.frcteam2910.c2019.autonomous.AutonomousSelector;
import org.frcteam2910.c2019.commands.*;
import org.frcteam2910.c2019.subsystems.*;
import org.frcteam2910.common.robot.commands.ZeroFieldOrientedCommand;
import org.frcteam2910.common.robot.input.DPadButton;
import org.frcteam2910.common.robot.input.XboxController;

public class OI {
    public final XboxController primaryController = new XboxController(0);

    public final XboxController secondaryController = new XboxController(1);

    public OI() {
        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        primaryController.getRightXAxis().setScale(0.45);
    }

    public void bindButtons(AutonomousSelector autonomousSelector) {
        primaryController.getRightTriggerAxis().whenPressed(new ExtendHatchPlacerCommand());
        {
            CommandGroup group = new CommandGroup();
            group.addSequential(new ReleaseHatchPanelCommand());
            group.addSequential(new InstantCommand(() -> HatchPlacerSubsystem.getInstance().extendPlacer()));
            group.addSequential(new WaitCommand(0.25));
            group.addSequential(new RetractHatchPlacerCommand());
            group.addSequential(new InstantCommand(() -> HatchPlacerSubsystem.getInstance().retractPlacer()));
            primaryController.getRightTriggerAxis().whenReleased(group);
        }

        primaryController.getAButton().whileHeld(new SetRobotPitchCommand(Math.toRadians(12.0)));

        {
            Command doTheThingCommand = new DoTheThingCommand(true, true);
            primaryController.getRightBumperButton().whenPressed(doTheThingCommand);
            primaryController.getRightBumperButton().whenReleased(new RetractHatchPlacerCommand());
            primaryController.getRightBumperButton().whenReleased(new InstantCommand(doTheThingCommand::cancel));
            primaryController.getRightBumperButton().whenReleased(new ConditionalCommand(new InstantCommand(() -> {
                autonomousSelector.getHybridQueue().remove().start();
            })) {
                @Override
                protected boolean condition() {
                    System.out.printf("Checking %d%n", autonomousSelector.getHybridQueue().size());
                    return DriverStation.getInstance().isAutonomous() && !autonomousSelector.getHybridQueue().isEmpty();
                }
            });
        }
        {
            Command doTheThingCommand = new DoTheThingCommand(false, true);
            primaryController.getLeftBumperButton().whenPressed(doTheThingCommand);
            primaryController.getLeftBumperButton().whenReleased(new RetractHatchPlacerCommand());
            primaryController.getLeftBumperButton().whenReleased(new InstantCommand(doTheThingCommand::cancel));
            primaryController.getLeftBumperButton().whenReleased(new ConditionalCommand(new InstantCommand(() -> {
                autonomousSelector.getHybridQueue().remove().start();
            })) {
                @Override
                protected boolean condition() {
                    System.out.printf("Checking %d%n", autonomousSelector.getHybridQueue().size());
                    return DriverStation.getInstance().isAutonomous() && !autonomousSelector.getHybridQueue().isEmpty();
                }
            });
        }

        primaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(
                new SetHatchFloorGathererAngleCommand(HatchFloorGathererSubsystem.getInstance().getMaxAngle()));
        primaryController.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(new SetHatchFloorGathererAngleCommand(Math.toRadians(90.0)));
        primaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(new SetHatchFloorGathererAngleCommand(0.0));
        primaryController.getDPadButton(DPadButton.Direction.LEFT).whenPressed(new RetractKickstandCommand());

        // Field oriented zero
        primaryController.getBackButton().whenPressed(new ZeroFieldOrientedCommand(DrivetrainSubsystem.getInstance()));

        // Climbing
        primaryController.getStartButton().whenPressed(new InstantCommand(new Runnable() {
            private SendableChooser<Integer> climbModeSendable = new SendableChooser<>();

            {
                ShuffleboardTab climbTab = Shuffleboard.getTab("Climbing");

                climbModeSendable.setName("Climb Mode");
                climbModeSendable.setDefaultOption("Normal", 0);
                climbModeSendable.addOption("Overhang", 1);
                climbModeSendable.addOption("MadTown", 2);
                climbTab.add(climbModeSendable);
            }

            @Override
            public void run() {
                switch (climbModeSendable.getSelected()) {
                    case 0:
                        new ClimbCommand().start();
                        break;
                    case 1:
                        new OverhangCommand(Math.toRadians(80.0), false).start();
                        break;
                    case 2:
                        new OverhangCommand(CargoArmSubsystem.getInstance().getMaxAngle(), true).start();
                        break;
                }
            }
        }));
        primaryController.getYButton().whenPressed(new AbortClimbCommand());


        // Cargo arm top position
        secondaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(
                new SetArmAngleCommand(CargoArmSubsystem.CARGO_SHIP_SCORE_ANGLE));
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

