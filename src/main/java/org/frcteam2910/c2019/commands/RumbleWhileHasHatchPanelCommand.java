package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.Robot;
import org.frcteam2910.c2019.subsystems.HatchPlacerSubsystem;

public class RumbleWhileHasHatchPanelCommand extends Command {

    @Override
    protected void execute() {
        if(HatchPlacerSubsystem.getInstance().getLeftLimitSwitch() && HatchPlacerSubsystem.getInstance().getRightLimitSwitch()) {
            Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
            Robot.getOi().secondaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        } else {
            Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
            Robot.getOi().secondaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        }
    }

    @Override
    protected void end() {
        Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        Robot.getOi().secondaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
