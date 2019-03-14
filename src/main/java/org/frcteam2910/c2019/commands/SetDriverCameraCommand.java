package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.frcteam2910.c2019.subsystems.VisionSubsystem;
import org.frcteam2910.c2019.vision.api.Gamepiece;

public class SetDriverCameraCommand extends InstantCommand {
    public SetDriverCameraCommand(Gamepiece gamepiece) {
        super(() -> VisionSubsystem.getInstance().setActiveDriverCamera(gamepiece));

        requires(VisionSubsystem.getInstance());
    }
}
