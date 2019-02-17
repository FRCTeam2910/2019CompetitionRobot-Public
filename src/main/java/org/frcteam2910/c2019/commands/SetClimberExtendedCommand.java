package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.frcteam2910.c2019.subsystems.ClimberSubsystem;

public class SetClimberExtendedCommand extends InstantCommand {
    private final boolean extended;

    public SetClimberExtendedCommand(boolean extended) {
        this.extended = extended;

        requires(ClimberSubsystem.getInstance());
    }


    @Override
    protected void initialize() {
        ClimberSubsystem.getInstance().setClimberExtended(extended);
    }
}
