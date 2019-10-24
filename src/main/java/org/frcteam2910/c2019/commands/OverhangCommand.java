package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.frcteam2910.c2019.subsystems.CargoArmSubsystem;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;

public class OverhangCommand extends CommandGroup {
    private static final double DRIVE_SPEED = 0.25;
    private static final double INTAKE_SPEED = 0.75;
    private static final double TARGET_PITCH = Math.toRadians(-5.0);

    public OverhangCommand(double armAngle, boolean moveWhileArmMoves) {
        addSequential(new GrabHatchPanelCommand());
        addParallel(new SetClimberExtendedCommand(true));
        addParallel(new DriveCommand(new Vector2(DRIVE_SPEED, 0.0), 0.0, false));
        addParallel(new SetBottomCargoRollerSpeedCommand(INTAKE_SPEED));
        addParallel(new ExtendKickstandCommand());
        addParallel(new CorrectPitchCommand(TARGET_PITCH, true));

        addSequential(new WaitCommand(0.5));
        addSequential(new WaitForFrontWheelsToExceedCurrentCommand(10.0));
        if (moveWhileArmMoves) {
            addSequential(new WaitForFrontWheelsToDriveDistanceCommand(10.0));
        } else {
            addSequential(new WaitForFrontWheelsToDriveDistanceCommand(4.0));
        }
        if (moveWhileArmMoves) {
            CommandGroup group = new CommandGroup();
            group.addSequential(new WaitForFrontWheelsToDriveDistanceCommand(4.0));
            group.addSequential(new SetClimberExtendedCommand(false));

            addParallel(group);
        } else {
            addParallel(new DriveCommand(new Vector2(MathUtils.EPSILON, 0.0), 0.0, false));
        }
        addSequential(new SetArmAngleCommand(armAngle));
//        addSequential(new InstantCommand(() -> {
//            DrivetrainSubsystem.getInstance().getCurrentCommand().cancel();
//        }));

        addParallel(new DriveCommand(new Vector2(DRIVE_SPEED, 0.0), 0.0, false));
        addSequential(new WaitForFrontWheelsToDriveDistanceCommand(12.0), 5.0);
        addSequential(new DriveCommand(Vector2.ZERO, 0.0, false));
    }
}
