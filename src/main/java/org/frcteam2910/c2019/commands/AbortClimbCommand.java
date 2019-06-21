package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AbortClimbCommand extends CommandGroup {
    public AbortClimbCommand() {
        this.addParallel(new SetClimberExtendedCommand(false));
        this.addParallel(new SetBottomCargoRollerSpeedCommand(0.3));
    }
}
