package org.frcteam2910.c2019;

import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.robot.commands.ZeroFieldOrientedCommand;
import org.frcteam2910.common.robot.input.XboxController;

public class OI {
    public final XboxController controller = new XboxController(0);

    public OI() {
        controller.getLeftXAxis().setInverted(true);
    }

    public void bindButtons() {
        controller.getBackButton().whenPressed(new ZeroFieldOrientedCommand(DrivetrainSubsystem.getInstance()));
    }
}
