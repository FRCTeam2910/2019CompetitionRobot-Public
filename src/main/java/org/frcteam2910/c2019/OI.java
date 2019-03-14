package org.frcteam2910.c2019;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.frcteam2910.c2019.commands.*;
import org.frcteam2910.c2019.subsystems.CargoArmSubsystem;
import org.frcteam2910.c2019.commands.SetHatchFloorGathererAngleCommand;
import org.frcteam2910.c2019.commands.SetHatchFloorGathererIntakeSpeedCommand;
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
    }

    public void bindButtons() {
        primaryController.getRightTriggerAxis().whenPressed(new ExtendHatchPlacerCommand());
        {
            CommandGroup group = new CommandGroup();
            group.addSequential(new ReleaseHatchPanelCommand());
            group.addSequential(new WaitCommand(0.1));
            group.addSequential(new RetractHatchPlacerCommand());
            primaryController.getRightTriggerAxis().whenReleased(group);
        }

        primaryController.getXButton().whenPressed(new SetDriverCameraCommand(Gamepiece.CARGO));
        primaryController.getYButton().whenPressed(new SetDriverCameraCommand(Gamepiece.HATCH_PANEL));

        primaryController.getAButton().whenPressed(new SetClimberExtendedCommand(true));
        primaryController.getAButton().whenReleased(new SetClimberExtendedCommand(false));

        primaryController.getRightBumperButton().whenPressed(new SetArmAngleCommand(CargoArmSubsystem.getInstance().getMaxAngle()));
        primaryController.getRightBumperButton().whileHeld(new VisionTargetCommand(Gamepiece.HATCH_PANEL));
        primaryController.getLeftBumperButton().whenPressed(new SetArmAngleCommand(CargoArmSubsystem.getInstance().getMaxAngle()));
        primaryController.getLeftBumperButton().whileHeld(new VisionTargetCommand(Gamepiece.CARGO));

        primaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(
                new SetHatchFloorGathererAngleCommand(HatchFloorGathererSubsystem.getInstance().getMaxAngle()));
        primaryController.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(new SetHatchFloorGathererAngleCommand(Math.toRadians(90.0)));
        primaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(new SetHatchFloorGathererAngleCommand(0.0));
        primaryController.getDPadButton(DPadButton.Direction.LEFT).whenPressed(new RetractKickstandCommand());

        // Field oriented zero
        primaryController.getBackButton().whenPressed(new ZeroFieldOrientedCommand(DrivetrainSubsystem.getInstance()));

        // Climbing
        primaryController.getStartButton().whenPressed(new ClimbCommand());
        primaryController.getStartButton().whenReleased(new AbortClimbCommand());


        // Cargo arm top position
        secondaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(new SetArmAngleCommand(
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
            group.addSequential(new GrabHatchPanelCommand());
            group.addSequential(new WaitCommand(0.1));
            group.addSequential(new RetractHatchPlacerCommand());
            secondaryController.getRightTriggerAxis().whenReleased(group);
        }
        secondaryController.getRightBumperButton().whileHeld(new GrabHatchFromFloorPart1Command());
        secondaryController.getRightBumperButton().whenReleased(new GrabHatchFromFloorPart2Command());

        secondaryController.getAButton().whenPressed(new SetHatchFloorGathererAngleCommand(
                HatchFloorGathererSubsystem.getInstance().getMaxAngle()));
    }
}

