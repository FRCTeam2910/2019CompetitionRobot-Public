package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.frcteam2910.c2019.subsystems.ClimberSubsystem;

public class ExtendKickstandCommand extends InstantCommand {
    public ExtendKickstandCommand() {
        super(() -> ClimberSubsystem.getInstance().extendKickstand());
    }
}
