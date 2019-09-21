package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.*;
import org.frcteam2910.c2019.subsystems.CargoArmSubsystem;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2019.subsystems.HatchPlacerSubsystem;
import org.frcteam2910.c2019.subsystems.Superstructure;
import org.frcteam2910.common.math.Vector2;

public class DoTheThingCommand extends CommandGroup {
    private static final double PICKUP_MAX_SPEED = 0.65;
    private static final double PLACE_MAX_SPEED = 0.5;
    private static final double PLACE_DISTANCE = 22.0;
    private static final double PLACE_HORIZ_DISTANCE = 1.0;

    public DoTheThingCommand() {
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
        placeGroup.addSequential(new ImprovedVisionPlaceCommand(PLACE_MAX_SPEED, (distanceFromTarget, horizDistance) -> {
            return distanceFromTarget < PLACE_DISTANCE && Math.abs(horizDistance) < PLACE_HORIZ_DISTANCE;
        }));
        placeGroup.addSequential(new InstantCommand(() -> HatchPlacerSubsystem.getInstance().extendPlacer())); // TODO
        placeGroup.addSequential(new ReleaseHatchPanelCommand());
        placeGroup.addSequential(new WaitCommand(0.2)); // TODO
        placeGroup.addSequential(new RetractHatchPlacerCommand());
        placeGroup.addSequential(new InstantCommand(() -> HatchPlacerSubsystem.getInstance().retractPlacer())); // TODO
        placeGroup.addSequential(new DriveCommand(new Vector2(0.5, 0.0), 0.0, false), 0.25);

        CommandGroup pickupGroup = new CommandGroup();
        pickupGroup.setRunWhenDisabled(true);
        pickupGroup.addSequential(new ExtendHatchPlacerCommand());
        pickupGroup.addSequential(new ReleaseHatchPanelCommand());
        pickupGroup.addSequential(new ImprovedVisionPlaceCommand(PICKUP_MAX_SPEED, (a, b) -> HatchPlacerSubsystem.getInstance().hasHatch(), 36.0, false));
        //pickupGroup.addSequential(new InstantCommand(() -> Superstructure.getInstance().getGyroscope().setAdjustmentAngle(Superstructure.getInstance().getGyroscope().getUnadjustedAngle())));
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
}
