package org.frcteam2910.c2019.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2019.vision.api.Gamepiece;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Vector2;

public class GotoTargetCommand extends Command {
    private final Gamepiece gamepiece;

    private PidController forwardsController = new PidController(new PidConstants(0.0, 0.0, 0.0));
    private PidController strafeController = new PidController(new PidConstants(0.0, 0.0, 0.0));
    private PidController angleController = new PidController(new PidConstants(0.1, 0.0, 0.0));

    private NetworkTable coprocessorTable = NetworkTableInstance.getDefault().getTable("vision-coprocessor");
    private NetworkTableEntry activeEntry = coprocessorTable.getEntry("active");
    private NetworkTableEntry cameraEntry = coprocessorTable.getEntry("camera");
    private NetworkTableEntry xEntry = coprocessorTable.getEntry("x");
    private NetworkTableEntry yEntry = coprocessorTable.getEntry("y");
    private NetworkTableEntry angleEntry = coprocessorTable.getEntry("angle");

    public GotoTargetCommand(Gamepiece gamepiece) {
        this.gamepiece = gamepiece;

        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void initialize() {
        cameraEntry.setNumber(gamepiece.ordinal());
        activeEntry.setBoolean(true);

        forwardsController.setSetpoint(0.0);
        strafeController.setSetpoint(0.0);
        angleController.setSetpoint(0.0);
        angleController.setInputRange(0.0, 2.0 * Math.PI);
        angleController.setContinuous(true);
    }

    double lastTime = 0.0;

    @Override
    protected void execute() {
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        lastTime = time;

        double x = xEntry.getDouble(Double.NaN);
        double y = yEntry.getDouble(Double.NaN);
        double angle = angleEntry.getDouble(Double.NaN);

        double forwardOutput = forwardsController.calculate(x, dt);
        double strafeOutput = strafeController.calculate(y, dt);
        double angleOutput = angleController.calculate(angle, dt);

        DrivetrainSubsystem.getInstance().holonomicDrive(new Vector2(forwardOutput, strafeOutput), -angleOutput, false);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        activeEntry.setBoolean(false);
    }
}
