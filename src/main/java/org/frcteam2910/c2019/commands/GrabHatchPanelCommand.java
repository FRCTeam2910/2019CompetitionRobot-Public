package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.frcteam2910.c2019.subsystems.HatchPlacerSubsystem;

public class GrabHatchPanelCommand extends InstantCommand {
    public GrabHatchPanelCommand() {
        super(() -> HatchPlacerSubsystem.getInstance().grab());

        this.setRunWhenDisabled(true);
    }
}
