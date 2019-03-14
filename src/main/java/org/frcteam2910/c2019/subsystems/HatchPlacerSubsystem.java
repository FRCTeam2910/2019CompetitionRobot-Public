package org.frcteam2910.c2019.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import org.frcteam2910.c2019.RobotMap;
import org.frcteam2910.common.robot.subsystems.Subsystem;

public class HatchPlacerSubsystem extends Subsystem {
    private static final double CAN_UPDATE_RATE = 50.0;

    private static final HatchPlacerSubsystem instance = new HatchPlacerSubsystem();

    private Solenoid extenderSolenoid = new Solenoid(RobotMap.HATCH_EXTENDER_SOLENOID_MODULE,
            RobotMap.HATCH_EXTENDER_SOLENOID_CHANNEL);
    private Solenoid grabberSolenoid = new Solenoid(RobotMap.HATCH_GRABBER_SOLENOID_MODULE,
            RobotMap.HATCH_GRABBER_SOLENOID_CHANNEL);

    private final Object canLock = new Object();
    private boolean extendedChanged = true;
    private boolean extended = false;
    private boolean releasedChanged = true;
    private boolean released = false;

    private Notifier canUpdateThread = new Notifier(() -> {
        boolean localExtended;
        boolean localExtendedChanged;
        boolean localGrabbing;
        boolean localGrabbingChanged;
        synchronized (canLock) {
            localExtended = extended;
            localExtendedChanged = extendedChanged;
            extendedChanged = false;

            localGrabbing = released;
            localGrabbingChanged = releasedChanged;
            releasedChanged = false;
        }

        if (localExtendedChanged) {
            extenderSolenoid.set(localExtended);
        }

        if (localGrabbingChanged) {
            grabberSolenoid.set(localGrabbing);
        }
    });


    private HatchPlacerSubsystem() {
        canUpdateThread.startPeriodic(1.0 / CAN_UPDATE_RATE);
    }

    public static HatchPlacerSubsystem getInstance() {
        return instance;
    }

    public void extend() {
        synchronized (canLock){
            extendedChanged = !extended;
            extended = true;
        }
    }

    public void retract() {
        synchronized (canLock) {
            extendedChanged = extended;
            extended = false;
        }
    }

    public void grab() {
        synchronized (canLock) {
            releasedChanged = released;
            released = false;
        }
    }

    public void release() {
        synchronized (canLock) {
            releasedChanged = !released;
            released = true;
        }
    }

    @Override
    public void outputToSmartDashboard() { }

    @Override
    public void stop() { }

    @Override
    public void zeroSensors() { }

    @Override
    protected void initDefaultCommand() { }
}
