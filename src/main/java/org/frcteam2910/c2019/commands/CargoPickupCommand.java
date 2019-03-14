package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.CargoGrabberSubsystem;

public class CargoPickupCommand extends Command {
    private double topSpeed;
    private double bottomSpeed;

    public CargoPickupCommand(double speed) {
        this(speed, speed);
    }

    public CargoPickupCommand(double topSpeed, double bottomSpeed) {
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;

        requires(CargoGrabberSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        CargoGrabberSubsystem.getInstance().setTopIntakeSpeed(topSpeed);
        CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(bottomSpeed);
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
