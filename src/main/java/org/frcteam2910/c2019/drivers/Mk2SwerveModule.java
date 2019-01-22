package org.frcteam2910.c2019.drivers;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Spark;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;

import java.util.concurrent.atomic.AtomicLong;

public class Mk2SwerveModule extends SwerveModule {
    private static final double ANALOG_INPUT_MAX_VOLTAGE = 4.95;
    private static final PidConstants ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);
    private static final double DRIVE_TICKS_PER_INCH = (1.0 / 6.71) * (Math.PI * 4.0);

    private static final double CAN_UPDATE_RATE = 200.0;

    private final double angleOffset;

    private Spark angleMotor;
    private AnalogInput angleEncoder;
    private CANSparkMax driveMotor;
    private CANEncoder driveEncoder;

    private AtomicLong driveEncoderValue = new AtomicLong();
    private AtomicLong driveOutputValue = new AtomicLong();

    private Notifier canUpdateNotifier = new Notifier(() -> {
        driveEncoderValue.set(Double.doubleToRawLongBits(driveEncoder.getPosition()));
        driveMotor.set(Double.longBitsToDouble(driveOutputValue.get()));
    });

    private PidController angleController = new PidController(ANGLE_CONSTANTS);

    public Mk2SwerveModule(Vector2 modulePosition, double angleOffset,
                           Spark angleMotor, CANSparkMax driveMotor, AnalogInput angleEncoder) {
        super(modulePosition);
        this.angleOffset = angleOffset;
        this.angleMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.driveMotor = driveMotor;
        this.driveEncoder = new CANEncoder(driveMotor);

        angleController.setInputRange(0.0, 2.0 * Math.PI);
        angleController.setContinuous(true);
        angleController.setOutputRange(-0.5, 0.5);

        canUpdateNotifier.startPeriodic(1.0 / CAN_UPDATE_RATE);
    }

    @Override
    protected double readAngle() {
        double angle = (1.0 - angleEncoder.getVoltage() / ANALOG_INPUT_MAX_VOLTAGE) * 2.0 * Math.PI + angleOffset;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    @Override
    protected double readDistance() {
        return Double.longBitsToDouble(driveEncoderValue.get()) / DRIVE_TICKS_PER_INCH;
    }

    @Override
    protected void setTargetAngle(double angle) {
        angleController.setSetpoint(angle);
    }

    @Override
    protected void setDriveOutput(double output) {
        driveOutputValue.set(Double.doubleToRawLongBits(output));
    }

    @Override
    public void updateState(double dt) {
        super.updateState(dt);

        angleMotor.set(angleController.calculate(getCurrentAngle(), dt));
    }
}
