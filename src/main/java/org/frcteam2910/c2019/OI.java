package org.frcteam2910.c2019;

import org.frcteam2910.c2019.commands.RotateHatchPlacerCommand;
import org.frcteam2910.c2019.commands.*;
import org.frcteam2910.c2019.subsystems.CargoArmSubsystem;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.robot.commands.ZeroFieldOrientedCommand;
import org.frcteam2910.common.robot.input.DPadButton;
import org.frcteam2910.common.robot.input.XboxController;

public class OI {
    public final XboxController primaryController = new XboxController(0);

    private final XboxController secondaryController = new XboxController(1);

    public OI() {
        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);
    }

    public void bindButtons() {
        primaryController.getLeftTriggerAxis().whileHeld(new CargoPickupCommand(-0.5));
        primaryController.getRightTriggerAxis().whileHeld(new RotateHatchPlacerCommand(-1.0));

        primaryController.getBackButton().whenPressed(new ZeroFieldOrientedCommand(DrivetrainSubsystem.getInstance()));

        primaryController.getAButton().whenPressed(new ClimbCommand());
        primaryController.getAButton().whenReleased(new SetClimberExtendedCommand(false));

        primaryController.getYButton().whenPressed(new ExtendHatchPlacerCommand());
        primaryController.getYButton().whenReleased(new RetractHatchPlacerCommand());

        primaryController.getXButton().whenPressed(new RotateHatchPlacerCommand(0.3));

        // Cargo arm top position
        secondaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(new SetArmAngleCommand(CargoArmSubsystem.MAX_ANGLE));
        // Cargo arm cargo ship height
        secondaryController.getDPadButton(DPadButton.Direction.LEFT).whenPressed(new SetArmAngleCommand(Math.toRadians(100.0)));
        // Cargo arm rocket & climb height
        secondaryController.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(new SetArmAngleCommand(Math.toRadians(65.0)));
        // Cargo arm bottom position
        secondaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(new SetArmAngleCommand(0.0));
    }
}

