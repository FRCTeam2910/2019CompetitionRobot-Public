package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.frcteam2910.c2019.subsystems.ClimberSubsystem;

public class RetractKickstandCommand extends InstantCommand {
    public RetractKickstandCommand() {
        super(() -> ClimberSubsystem.getInstance().retractKickstand());
    }
}
