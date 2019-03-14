package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.frcteam2910.c2019.subsystems.HatchPlacerSubsystem;

public class ExtendHatchPlacerCommand extends InstantCommand {

	@Override
	protected void initialize() {
		HatchPlacerSubsystem.getInstance().extend();
	}
}
