package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.HatchFloorGathererSubsystem;

public class WaitForHatchArmAngleCommand extends Command {
    @Override
    protected boolean isFinished() {
        return HatchFloorGathererSubsystem.getInstance().isAtTargetAngle();
    }
}
