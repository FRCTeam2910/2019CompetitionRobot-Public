package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.HatchPlacerSubsystem;

public class HatchPlacerSoftIntakeCommand extends Command {
    private double speed;

    public HatchPlacerSoftIntakeCommand(double speed) {
        this.speed = speed;
        requires(HatchPlacerSubsystem.getInstance());
    }

    @Override
    protected void execute() {
//        if(HatchPlacerSubsystem.getInstance().hasHatchPanel()) {
            HatchPlacerSubsystem.getInstance().activatePlacerMotor(speed);
//        } else {
//            HatchPlacerSubsystem.getInstance().activatePlacerMotor(0);
//        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
