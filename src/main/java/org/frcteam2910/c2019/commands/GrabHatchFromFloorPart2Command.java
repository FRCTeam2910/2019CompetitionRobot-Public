package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.frcteam2910.c2019.subsystems.HatchFloorGathererSubsystem;

public class GrabHatchFromFloorPart2Command extends CommandGroup {
    public GrabHatchFromFloorPart2Command() {
        addParallel(new SetHatchFloorGathererIntakeSpeedCommand(0.3, false), 0.5);
        addSequential(new SetHatchFloorGathererAngleCommand(Math.toRadians(90.0)));
        addSequential(new ExtendHatchPlacerCommand());
        addSequential(new GrabHatchPanelCommand());
        addSequential(new WaitCommand(0.25));
        addParallel(new SetHatchFloorGathererIntakeSpeedCommand(-0.3, false), 0.5);
        addSequential(new SetHatchFloorGathererAngleCommand(Math.toRadians(45.0)));
        addSequential(new SetHatchFloorGathererAngleCommand(HatchFloorGathererSubsystem.getInstance().getMaxAngle()));
        addSequential(new RetractHatchPlacerCommand());
    }
}
