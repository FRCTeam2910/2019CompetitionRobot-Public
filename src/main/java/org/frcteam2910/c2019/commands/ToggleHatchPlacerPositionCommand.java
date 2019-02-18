package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.frcteam2910.c2019.subsystems.HatchPlacerSubsystem;

public class ToggleHatchPlacerPositionCommand extends InstantCommand {

	public ToggleHatchPlacerPositionCommand() {
		requires(HatchPlacerSubsystem.getInstance());
	}

	@Override
	protected void initialize() {
		if(HatchPlacerSubsystem.getInstance().getSolenoidPosition()){
			HatchPlacerSubsystem.getInstance().retractPlacer();
		} else {
			HatchPlacerSubsystem.getInstance().extendPlacer();
		}
	}
}
