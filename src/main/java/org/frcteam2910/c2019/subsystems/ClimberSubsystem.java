package org.frcteam2910.c2019.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import org.frcteam2910.c2019.RobotMap;
import org.frcteam2910.common.robot.subsystems.Subsystem;

public class ClimberSubsystem extends Subsystem {
    private static final ClimberSubsystem instance = new ClimberSubsystem();

    private Solenoid climberSolenoid = new Solenoid(RobotMap.CLIMBER_SOLENOID_MODULE,
            RobotMap.CLIMBER_SOLENOID_CHANNEL);

    private final Object canLock = new Object();
    private boolean climberStateChanged = false;
    private boolean climberExtended = true;

    private Notifier canUpdateThread = new Notifier(() -> {
        boolean localClimberStateChanged;
        boolean localClimberExtended;

        synchronized (canLock) {
            localClimberStateChanged = climberStateChanged;
            localClimberExtended = climberExtended;
            climberStateChanged = false;
        }

        if (localClimberStateChanged) {
            climberSolenoid.set(localClimberExtended);
        }
    });

    private ClimberSubsystem() {
        canUpdateThread.startPeriodic(0.02);
    }

    public static ClimberSubsystem getInstance() {
        return instance;
    }

    public void setClimberExtended(boolean extended) {
        synchronized (canLock) {
            climberStateChanged = true;
            climberExtended = extended;
        }
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void initDefaultCommand() {

    }
}
