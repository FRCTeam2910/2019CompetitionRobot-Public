package org.frcteam2910.c2019.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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
    private Solenoid placerSolenoid = new Solenoid(RobotMap.HATCH_PLACER_SOLENOID_MODULE,
            RobotMap.HATCH_PLACER_SOLENOID_CHANNEL);

    private DigitalInput leftLimitSwitch = new DigitalInput(RobotMap.HATCH_GRABBER_LIMIT_SWITCH_LEFT);
    private DigitalInput rightLimitSwitch = new DigitalInput(RobotMap.HATCH_GRABBER_LIMIT_SWITCH_RIGHT);

    private final Object canLock = new Object();
    private boolean extendedChanged = true;
    private boolean extended = false;
    private boolean releasedChanged = true;
    private boolean released = false;
    private boolean placingChanged = true;
    private boolean placing = false;

    private Notifier canUpdateThread = new Notifier(() -> {
        boolean localExtended;
        boolean localExtendedChanged;
        boolean localGrabbing;
        boolean localGrabbingChanged;
        boolean localPlacing;
        boolean localPlacingChanged;
        synchronized (canLock) {
            localExtended = extended;
            localExtendedChanged = extendedChanged;
            extendedChanged = false;

            localGrabbing = released;
            localGrabbingChanged = releasedChanged;
            releasedChanged = false;

            localPlacing = placing;
            localPlacingChanged = placingChanged;
            placingChanged = false;
        }

        if (localExtendedChanged) {
            extenderSolenoid.set(localExtended);
        }

        if (localGrabbingChanged) {
            grabberSolenoid.set(localGrabbing);
        }

        if (localPlacingChanged) {
            placerSolenoid.set(localPlacing);
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

    public void retractPlacer() {
        synchronized (canLock) {
            placingChanged = placing;
            placing = false;
        }
    }

    public void extendPlacer() {
        synchronized (canLock) {
            placingChanged = !placing;
            placing = true;
        }
    }

    public boolean getRightLimitSwitch() {
        return !rightLimitSwitch.get();
    }

    public boolean getLeftLimitSwitch() {
        return !leftLimitSwitch.get();
    }

    public boolean hasHatch() {
        return getLeftLimitSwitch() || getRightLimitSwitch();
    }

    @Override
    public void outputToSmartDashboard() { }

    @Override
    public void stop() { }

    @Override
    public void zeroSensors() { }

    @Override
    protected void initDefaultCommand() { }

    public boolean isReleased() {
        synchronized (canLock) {
            return released;
        }
    }
}
