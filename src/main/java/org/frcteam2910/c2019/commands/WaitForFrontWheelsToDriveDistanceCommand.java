package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2019.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.drivers.SwerveModule;

public class WaitForFrontWheelsToDriveDistanceCommand extends Command {
    private final double distance;

    private double[] startingDistances;

    public WaitForFrontWheelsToDriveDistanceCommand(double distance) {
        this.distance = distance;
    }

    @Override
    protected void initialize() {
        startingDistances = new double[DrivetrainSubsystem.getInstance().getSwerveModules().length];
        for (int i = 0; i < startingDistances.length; i++) {
            startingDistances[i] = DrivetrainSubsystem.getInstance().getSwerveModules()[i].getCurrentDistance();
        }
    }

    @Override
    protected boolean isFinished() {
        double avgDistance = 0.0;
        int moduleCount = 0;
        for (int i = 0; i < startingDistances.length; i++) {
            SwerveModule module = DrivetrainSubsystem.getInstance().getSwerveModules()[i];
            if (module.getModulePosition().y > 0.0) {
                double delta = Math.abs(module.getCurrentDistance() - startingDistances[i]);

                avgDistance += delta;
                moduleCount++;
            }
        }

        if (moduleCount == 0) {
            return false;
        } else {
            SmartDashboard.putNumber("Distance Driven", avgDistance / moduleCount);
            return avgDistance / moduleCount > distance;
        }
    }
}
