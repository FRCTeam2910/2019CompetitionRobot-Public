package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.CargoGrabberSubsystem;

public class CargoPickupCommand extends Command {
    private double speed;

    public CargoPickupCommand(double speed) {
        this.speed = speed;

        requires(CargoGrabberSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        CargoGrabberSubsystem.getInstance().setIntakeSpeed(-speed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        CargoGrabberSubsystem.getInstance().stop();
    }
}
