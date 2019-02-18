package org.frcteam2910.c2019.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import org.frcteam2910.c2019.RobotMap;
import org.frcteam2910.c2019.commands.HatchPlacerSoftIntakeCommand;
import org.frcteam2910.common.robot.subsystems.Subsystem;

public class HatchPlacerSubsystem extends Subsystem {
    private static final HatchPlacerSubsystem instance = new HatchPlacerSubsystem();

    private static final double CAN_UPDATE_RATE = 50.0;
    private static final double HATCH_PLACER_SOFT_INTAKE_SPEED = 0.1;

    private final Object sensorLock = new Object();

    private Solenoid hatchPlacerSolenoid = new Solenoid(RobotMap.HATCH_PLACER_SOLENOID_MODULE_NUMBER,RobotMap.HATCH_PLACER_SOLENOID_CHANNEL);   //TODO: Find Real Values
    private CANSparkMax hatchPlacerMotor = new CANSparkMax(RobotMap.HATCH_PLACER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANDigitalInput hatchLimitSwitch = new CANDigitalInput(hatchPlacerMotor, CANDigitalInput.LimitSwitch.kForward, CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);

    private boolean isHatchPlacerExtended = false;
    private boolean limitSwitchEnabled = false;
    private double hatchPlacementMotorSpeed = 0;


    private Notifier canUpdateThread = new Notifier(() -> {
        boolean hatchLimitSwitchEnabled = hatchLimitSwitch.get();
        synchronized (sensorLock) {
            limitSwitchEnabled = hatchLimitSwitchEnabled;
        }

        boolean hatchPlacerExtended;
        synchronized (sensorLock) {
            hatchPlacerExtended = isHatchPlacerExtended;
        }
        hatchPlacerSolenoid.set(hatchPlacerExtended);

        double speed;
        synchronized (sensorLock) {
            speed = hatchPlacementMotorSpeed;
        }
        hatchPlacerMotor.set(speed);
    });


    private HatchPlacerSubsystem() {
        canUpdateThread.startPeriodic(1.0 / CAN_UPDATE_RATE);
    }

    public static HatchPlacerSubsystem getInstance() {
        return instance;
    }

    public boolean hasHatchPanel() {
        synchronized (sensorLock) {
            return limitSwitchEnabled;
        }
    }

    public void activatePlacerMotor(double speed) {
//        if (hasHatchPanel() && speed > 0) {
//            speed = 0;
//        }

        synchronized (sensorLock) {
            hatchPlacementMotorSpeed = speed;
        }
    }

    public boolean getSolenoidPosition() {
        synchronized (sensorLock) {
            return isHatchPlacerExtended;
        }
    }

    public void retractPlacer() {
        synchronized (sensorLock) {
            isHatchPlacerExtended = false;
        }
    }

    public void extendPlacer() {
        synchronized (sensorLock){
            isHatchPlacerExtended = true;
        }
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new HatchPlacerSoftIntakeCommand(HATCH_PLACER_SOFT_INTAKE_SPEED));
    }
}
