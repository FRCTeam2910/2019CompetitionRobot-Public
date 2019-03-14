package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.Robot;
import org.frcteam2910.c2019.subsystems.HatchFloorGathererSubsystem;

public class SetHatchFloorGathererIntakeSpeedCommand extends Command {
    private double speed;

    public SetHatchFloorGathererIntakeSpeedCommand(double speed) {
        this(speed, true);
    }

    public SetHatchFloorGathererIntakeSpeedCommand(double speed, boolean shouldRequire) {
        this.speed = speed;

        if (shouldRequire) {
            requires(HatchFloorGathererSubsystem.getInstance());
        }
    }

    @Override
    protected void initialize() {
        HatchFloorGathererSubsystem.getInstance().setTargetIntakeSpeed(speed);
    }

    @Override
    protected void execute() {
        // If we have a hatch panel, vibrate the secondary controller
        if (HatchFloorGathererSubsystem.getInstance().hasHatchPanel()) {
            Robot.getOi().secondaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        }
    }

    @Override
    protected void end() {
        Robot.getOi().secondaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);

        HatchFloorGathererSubsystem.getInstance().setTargetIntakeSpeed(0.0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
