package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.HatchPlacerSubsystem;

public class RotateHatchPlacerCommand extends Command {
	private double speed;

	public RotateHatchPlacerCommand(double speed) {
		this.speed = speed;
		requires(HatchPlacerSubsystem.getInstance());
	}

	@Override
	protected void initialize() {
		HatchPlacerSubsystem.getInstance().activatePlacerMotor(speed);
	}

	@Override
	protected void end() {
		HatchPlacerSubsystem.getInstance().activatePlacerMotor(0);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
