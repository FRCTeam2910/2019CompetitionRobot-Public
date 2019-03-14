package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.subsystems.CargoArmSubsystem;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.drivers.SwerveModule;

import java.util.ArrayList;
import java.util.List;

public class CorrectPitchCommand extends Command {
    private static final double WHEEL_CONTACT_WAIT = 0.5;

    private static final double WHEEL_CONTACT_SPEED = 5.0;

    private final double targetPitch;
    private final boolean pitchOnlyDown;

    private final Timer wheelContactTimer = new Timer();
    private boolean wheelContanctTimerRunning = false;

    private List<SwerveModule> frontModules = new ArrayList<>();
    private double[] lastModuleDistances;
    private double lastExecuteTime = Timer.getFPGATimestamp();


    public CorrectPitchCommand(double pitch, boolean pitchOnlyDown) {
        this.targetPitch = pitch;
        this.pitchOnlyDown = pitchOnlyDown;

        // Find all the swerve modules which are in the front and add them to the list
        for (SwerveModule module : DrivetrainSubsystem.getInstance().getSwerveModules()) {
            if (module.getModulePosition().x > 0.0) {
                frontModules.add(module);
            }
        }
        lastModuleDistances = new double[frontModules.size()];

        requires(CargoArmSubsystem.getInstance());
    }

    @Override
    protected void initialize() {
        wheelContactTimer.stop();
        wheelContactTimer.reset();
        wheelContanctTimerRunning = false;
        CargoArmSubsystem.getInstance().setTargetPitch(targetPitch, pitchOnlyDown);
        lastExecuteTime = Double.NaN;
    }

    @Override
    protected void execute() {
        double now = Timer.getFPGATimestamp();
        if (Double.isNaN(lastExecuteTime)) {
            lastExecuteTime = now;
            return;
        }
        double dt = now - lastExecuteTime;
        lastExecuteTime = now;

        double averageSpeed = 0.0;
        for (int i = 0; i < frontModules.size(); i++) {
            double distance = frontModules.get(i).getCurrentDistance();
            double speed = Math.abs((distance - lastModuleDistances[i]) / dt);
            lastModuleDistances[i] = distance;
            averageSpeed += speed;
        }
        averageSpeed /= frontModules.size();

        if (averageSpeed < WHEEL_CONTACT_SPEED) {
            if (!wheelContanctTimerRunning) {
                wheelContactTimer.start();
                wheelContanctTimerRunning = true;
            }
        } else {
            if (wheelContanctTimerRunning) {
                wheelContactTimer.stop();
                wheelContactTimer.reset();
                wheelContanctTimerRunning = false;
            }
        }
    }

    @Override
    protected void end() {
        wheelContactTimer.stop();
        CargoArmSubsystem.getInstance().disable();
    }

    @Override
    protected boolean isFinished() {
        return wheelContanctTimerRunning && wheelContactTimer.get() > WHEEL_CONTACT_WAIT;
    }
}
