package org.frcteam2910.c2019.commands;


import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.ArmSubsystem;


public class SetArmAngleCommand extends Command {
	private double angle;

	public SetArmAngleCommand(double angle) {
		this.angle = angle;
		requires(ArmSubsystem.getInstance());
	}

	@Override
	protected void initialize() {
		ArmSubsystem.getInstance().setTargetAngle(angle);
	}

	@Override
	protected boolean isFinished() {
		return ArmSubsystem.getInstance().isWithinTargetAngleRange(angle);
	}
}
