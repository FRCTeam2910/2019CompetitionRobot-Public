package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.ClimberSubsystem;
import org.frcteam2910.c2019.subsystems.Superstructure;
import org.frcteam2910.common.robot.drivers.NavX;

public class SetRobotPitchCommand extends Command {
    private final double targetPitch;

    public SetRobotPitchCommand(double targetPitch) {
        this.targetPitch = targetPitch;

        requires(ClimberSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        double currentPitch = Superstructure.getInstance().getGyroscope().getAxis(NavX.Axis.ROLL); // Roll is pitch
        if (currentPitch > targetPitch) {
            ClimberSubsystem.getInstance().setClimberExtended(false);
        } else {
            ClimberSubsystem.getInstance().setClimberExtended(true);
        }
    }

    @Override
    protected void end() {
        ClimberSubsystem.getInstance().setClimberExtended(false);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
