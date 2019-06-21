package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;

public class OverhangCommand extends CommandGroup {
    private static final double DRIVE_SPEED = 0.25;
    private static final double INTAKE_SPEED = 0.75;
    private static final double TARGET_PITCH = Math.toRadians(-5.0);

    public OverhangCommand() {
        addParallel(new SetClimberExtendedCommand(true));
        addParallel(new DriveCommand(new Vector2(DRIVE_SPEED, 0.0), 0.0, false));
        addParallel(new SetBottomCargoRollerSpeedCommand(INTAKE_SPEED));
        addParallel(new ExtendKickstandCommand());
        addParallel(new CorrectPitchCommand(TARGET_PITCH, true));

        addSequential(new WaitCommand(0.5));
        addSequential(new WaitForFrontWheelsToExceedCurrentCommand(5.0));
        addSequential(new WaitForFrontWheelsToDriveDistanceCommand(2.0));
        addParallel(new DriveCommand(new Vector2(MathUtils.EPSILON, 0.0), 0.0, false));
        addSequential(new SetArmAngleCommand(Math.toRadians(45.0)));
        addSequential(new InstantCommand(() -> {
            DrivetrainSubsystem.getInstance().getCurrentCommand().cancel();
        }));
//        addSequential(new DriveCommand(new Vector2(DRIVE_SPEED, 0.0), 0.0, false));
    }
}
