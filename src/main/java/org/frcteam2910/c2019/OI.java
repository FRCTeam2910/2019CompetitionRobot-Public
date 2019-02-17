package org.frcteam2910.c2019;

import org.frcteam2910.c2019.commands.*;
import org.frcteam2910.c2019.subsystems.CargoArmSubsystem;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.robot.commands.ZeroFieldOrientedCommand;
import org.frcteam2910.common.robot.input.DPadButton;
import org.frcteam2910.common.robot.input.XboxController;

public class OI {
    public final XboxController controller = new XboxController(0);

    public OI() {
        controller.getLeftXAxis().setInverted(true);
    }

    public void bindButtons() {
        controller.getBackButton().whenPressed(new ZeroFieldOrientedCommand(DrivetrainSubsystem.getInstance()));

        controller.getStartButton().whenPressed(new ClimbCommand());
        controller.getStartButton().whenReleased(new SetClimberExtendedCommand(false));

        controller.getDPadButton(DPadButton.Direction.UP).whenPressed(new SetArmAngleCommand(CargoArmSubsystem.MAX_ANGLE));
        controller.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(new SetArmAngleCommand(Math.toRadians(65.0)));
        controller.getDPadButton(DPadButton.Direction.DOWN).whenPressed(new SetArmAngleCommand(0.0));

        controller.getAButton().whenPressed(new CargoPickupCommand(0.3));
        controller.getBButton().whileHeld(new CargoPickupCommand(-1.0));
    }
}
