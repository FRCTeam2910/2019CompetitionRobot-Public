package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class GrabHatchFromFloorPart2Command extends CommandGroup {
    public GrabHatchFromFloorPart2Command() {
        addParallel(new SetHatchFloorGathererIntakeSpeedCommand(0.3, false), 0.5);
        addSequential(new SetHatchFloorGathererAngleCommand(Math.toRadians(90.0)));
//        addSequential(new WaitForHatchArmAngleCommand(), 0.5);
        addSequential(new GrabHatchPanelCommand());
        addSequential(new SetHatchFloorGathererAngleCommand(Math.toRadians(15.0)));
        addSequential(new RetractHatchPlacerCommand());
    }
}
