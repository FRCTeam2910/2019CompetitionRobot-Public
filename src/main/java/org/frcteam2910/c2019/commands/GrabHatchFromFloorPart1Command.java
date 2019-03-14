package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GrabHatchFromFloorPart1Command extends CommandGroup {
    public GrabHatchFromFloorPart1Command() {
        addSequential(new ReleaseHatchPanelCommand());
        addSequential(new ExtendHatchPlacerCommand());
        addSequential(new SetHatchFloorGathererAngleCommand(0.0));
        addSequential(new SetHatchFloorGathererIntakeSpeedCommand(0.3));
        // TODO: When we detect the hatch panel, notify the driver
    }
}
