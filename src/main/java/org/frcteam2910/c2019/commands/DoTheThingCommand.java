package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.*;
import org.frcteam2910.c2019.subsystems.CargoArmSubsystem;
import org.frcteam2910.c2019.subsystems.HatchPlacerSubsystem;
import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class DoTheThingCommand extends CommandGroup {
    private static final double RELEASE_DISTANCE = 21.5;
    private static final double RELEASE_HORIZ_DISTANCE = 1.25;

    public DoTheThingCommand(boolean placeOnRocket, boolean backupAfterPlace) {
        this.setRunWhenDisabled(true);
        // Setup
        addSequential(new ExtendHatchPlacerCommand());
        Command setupHatchMechanism = new ConditionalCommand((Command) null, new ReleaseHatchPanelCommand()) {
            @Override
            protected boolean condition() {
                return HatchPlacerSubsystem.getInstance().hasHatch();
            }
        };
        setupHatchMechanism.setRunWhenDisabled(true);
        addSequential(setupHatchMechanism);
        addSequential(new SetArmAngleCommand(CargoArmSubsystem.VISION_TARGET_ANGLE));

        CommandGroup placeGroup = new CommandGroup();
        placeGroup.setRunWhenDisabled(true);
        placeGroup.addSequential(new ImprovedVisionPlaceCommand(
                (distanceFromTarget, horizDistance) -> distanceFromTarget < RELEASE_DISTANCE && Math.abs(horizDistance) < RELEASE_HORIZ_DISTANCE,
                placeOnRocket ? DoTheThingCommand::chooseRocketTarget : DoTheThingCommand::chooseCargoShipTarget,
                new ITrajectoryConstraint[]{
                        new MaxVelocityConstraint(12.0 * 12.0),
                        new MaxAccelerationConstraint(7.5 * 12.0)
                }
        ));
        placeGroup.addSequential(new InstantCommand(() -> HatchPlacerSubsystem.getInstance().extendPlacer())); // TODO
        placeGroup.addSequential(new ReleaseHatchPanelCommand());
        placeGroup.addSequential(new WaitCommand(0.2)); // TODO
        placeGroup.addSequential(new RetractHatchPlacerCommand());
        placeGroup.addSequential(new InstantCommand(() -> HatchPlacerSubsystem.getInstance().retractPlacer())); // TODO
        if (backupAfterPlace) {
            placeGroup.addSequential(new DriveCommand(new Vector2(0.5, 0.0), 0.0, false), 0.25);
        }

        CommandGroup pickupGroup = new CommandGroup();
        pickupGroup.setRunWhenDisabled(true);
        pickupGroup.addSequential(new ExtendHatchPlacerCommand());
        pickupGroup.addSequential(new ReleaseHatchPanelCommand());
        pickupGroup.addSequential(new ImprovedVisionPlaceCommand(
                (a, b) -> HatchPlacerSubsystem.getInstance().hasHatch(),
                placeOnRocket ? DoTheThingCommand::chooseRocketTarget : DoTheThingCommand::chooseCargoShipTarget,
                new ITrajectoryConstraint[]{
                        new MaxVelocityConstraint(12.0 * 12.0),
                        new MaxAccelerationConstraint(11.0 * 12.0)
                }
        ));
        pickupGroup.addSequential(new GrabHatchPanelCommand());
        pickupGroup.addSequential(new WaitCommand(0.25)); // TODO
        pickupGroup.addSequential(new RetractHatchPlacerCommand());

        // Target
        Command cleanupHatchMechanism = new ConditionalCommand(placeGroup, pickupGroup) {
            @Override
            protected boolean condition() {
                return HatchPlacerSubsystem.getInstance().hasHatch();
            }
        };
        addSequential(cleanupHatchMechanism);
    }

    private static Rotation2 chooseCargoShipTarget(Rotation2 currentAngle) {
        if (!HatchPlacerSubsystem.getInstance().hasHatch()) {
            return Rotation2.ZERO;
        }

        double angle = currentAngle.toRadians();
        if (angle < Math.toRadians(135.0)) {
            return Rotation2.fromDegrees(90.0);
        } else if (angle < Math.toRadians(225.0)) {
            return Rotation2.fromDegrees(180.0);
        } else {
            return Rotation2.fromDegrees(270.0);
        }
    }

    private static Rotation2 chooseRocketTarget(Rotation2 currentAngle) {
        if (!HatchPlacerSubsystem.getInstance().hasHatch()) {
            return Rotation2.ZERO;
        }

        double angle = currentAngle.toRadians();
        if (angle < Math.toRadians(90.0)) {
            return Rotation2.fromDegrees(32.0);
        } else if (angle < Math.toRadians(180.0)) {
            return Rotation2.fromDegrees(148.0);
        } else if (angle < Math.toRadians(270.0)) {
            return Rotation2.fromDegrees(212.0);
        } else {
            return Rotation2.fromDegrees(328.0);
        }
    }
}
