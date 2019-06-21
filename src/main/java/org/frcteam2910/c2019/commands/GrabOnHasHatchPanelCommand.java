package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.HatchPlacerSubsystem;

public class GrabOnHasHatchPanelCommand extends Command {
    private static final double SIGNAL_LENGTH = 0.1;

    private double lastSignalTime = Double.NaN;

    @Override
    protected void initialize() {
        lastSignalTime = Double.NaN;
    }

    @Override
    protected void end() {
        HatchPlacerSubsystem.getInstance().grab();
    }

    @Override
    protected boolean isFinished() {
        boolean hasValidSignal = HatchPlacerSubsystem.getInstance().getLeftLimitSwitch() || HatchPlacerSubsystem.getInstance().getRightLimitSwitch();

        if (!Double.isNaN(lastSignalTime)) {
            if (lastSignalTime + SIGNAL_LENGTH < Timer.getFPGATimestamp()) {
                return true;
            } else if (!hasValidSignal){
                lastSignalTime = Double.NaN;
            }
        } else if (hasValidSignal){
            lastSignalTime = Timer.getFPGATimestamp();
        }

        return false;
    }

    @Override
    protected void interrupted() {

    }
}
