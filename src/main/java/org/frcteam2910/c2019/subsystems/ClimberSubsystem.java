package org.frcteam2910.c2019.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import org.frcteam2910.c2019.RobotMap;
import org.frcteam2910.common.robot.subsystems.Subsystem;

public class ClimberSubsystem extends Subsystem {
    private static final ClimberSubsystem instance = new ClimberSubsystem();

    private Solenoid climberSolenoid = new Solenoid(RobotMap.CLIMBER_SOLENOID_MODULE,
            RobotMap.CLIMBER_SOLENOID_CHANNEL);
    private Solenoid kickstandSolenoid = new Solenoid(RobotMap.KICKSTAND_SOLENOID_MODULE,
            RobotMap.KICKSTAND_SOLENOID_CHANNEL);

    private final Object canLock = new Object();
    private boolean climberStateChanged = false;
    private boolean climberExtended = false;
    private boolean kickstandStateChanged = true;
    private boolean kickstandExtended = false;

    private Notifier canUpdateThread = new Notifier(() -> {
        boolean localClimberStateChanged;
        boolean localClimberExtended;
        boolean localKickstandStateChanged;
        boolean localKickstandExtended;

        synchronized (canLock) {
            localClimberStateChanged = climberStateChanged;
            localClimberExtended = climberExtended;
            climberStateChanged = false;

            localKickstandStateChanged = kickstandStateChanged;
            localKickstandExtended = kickstandExtended;
            kickstandStateChanged = false;
        }

        if (localClimberStateChanged) {
            climberSolenoid.set(localClimberExtended);
        }

        // Kickstand is inverted
        if (localKickstandStateChanged) {
            kickstandSolenoid.set(!localKickstandExtended);
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
            climberStateChanged = climberExtended != extended;
            climberExtended = extended;
        }
    }

    public void extendKickstand() {
        synchronized (canLock) {
            kickstandStateChanged = !kickstandExtended;
            kickstandExtended = true;
        }
    }

    public void retractKickstand() {
        synchronized (canLock) {
            kickstandStateChanged = kickstandExtended;
            kickstandExtended = false;
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
