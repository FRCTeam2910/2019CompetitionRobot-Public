package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.CargoArmSubsystem;

public class CorrectPitchCommand extends Command {

    public CorrectPitchCommand() {
        requires(CargoArmSubsystem.getInstance());
    }

    @Override
    protected void initialize() {
        CargoArmSubsystem.getInstance().setTargetPitch(Math.toRadians(-5.0));
    }

    @Override
    protected void end() {
        CargoArmSubsystem.getInstance().disable();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
