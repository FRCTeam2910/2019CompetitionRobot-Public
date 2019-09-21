package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.CargoGrabberSubsystem;

public class SetBottomCargoRollerSpeedCommand extends Command {
    private final double speed;

    public SetBottomCargoRollerSpeedCommand(double speed) {
        this.speed = speed;

        requires(CargoGrabberSubsystem.getInstance());
    }

    @Override
    protected void initialize() {
        CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(speed);
    }

    @Override
    protected void end() {
        CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(0.0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
