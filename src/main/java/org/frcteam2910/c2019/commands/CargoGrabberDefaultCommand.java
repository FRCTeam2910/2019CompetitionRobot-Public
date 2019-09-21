package org.frcteam2910.c2019.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.c2019.Robot;
import org.frcteam2910.c2019.subsystems.CargoArmSubsystem;
import org.frcteam2910.c2019.subsystems.CargoGrabberSubsystem;
import org.frcteam2910.c2019.subsystems.Superstructure;
import org.frcteam2910.common.math.MathUtils;

public class CargoGrabberDefaultCommand extends Command {
    private static final double EJECT_SPEED_TOP = -1.0;
    private static final double EJECT_SPEED_BOTTOM = 0.0;

    private static final double EJECT_FRONT_CARGO_SHIP_SPEED_TOP = -0.6;
    private static final double EJECT_FRONT_CARGO_SHIP_SPEED_BOTTOM = 0.0;

    private static final double EJECT_ROCKET_SPEED_TOP = -0.2;
    private static final double EJECT_ROCKET_SPEED_BOTTOM = 1.0;

    private static final double INTAKE_AFTER_EJECT_TIME = 2.5;

    private static final double INTAKE_AFTER_EJECT_SPEED_TOP = 0.7;
    private static final double INTAKE_AFTER_EJECT_SPEED_BOTTOM = 0.0;

    private static final double INTAKE_SPEED_TOP = 0.7;
    private static final double INTAKE_SPEED_BOTTOM = 0.5;

    private static final double CENTERING_INTAKE_SPEED_TOP = 0.6;
    private static final double CENTERING_INTAKE_SPEED_BOTTOM = 0.5;

    private static final double SOFT_INTAKE_SPEED_TOP = 0.06;
    private static final double SOFT_INTAKE_SPEED_BOTTOM = 0.0;

    private final Timer timer = new Timer();

    public CargoGrabberDefaultCommand() {
        requires(CargoGrabberSubsystem.getInstance());
    }

    @Override
    protected void initialize() {
        timer.start();
    }

    @Override
    protected void execute() {
        if (Robot.getOi().primaryController.getLeftTriggerAxis().get() > 0.1) {
            double angle = Superstructure.getInstance().getGyroscope().getAngle().toRadians();

            if (MathUtils.epsilonEquals(CargoArmSubsystem.getInstance().getCurrentAngle(),
                    CargoArmSubsystem.ROCKET_SCORE_ANGLE, Math.toRadians(5.0))) {
                // Placing in the rocket uses different speeds because sometimes when we eject cargo it rides up the
                // side of the rocket.
                CargoGrabberSubsystem.getInstance().setTopIntakeSpeed(EJECT_ROCKET_SPEED_TOP);
                CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(EJECT_ROCKET_SPEED_BOTTOM);
            } else if (angle < Math.toRadians(30.0) || angle > Math.toRadians(360.0 - 30.0)) {
                // When we are scoring in the front of the cargo ship, the ejection speed is different so the cargo
                // doesn't launch as far.
                CargoGrabberSubsystem.getInstance().setTopIntakeSpeed(EJECT_FRONT_CARGO_SHIP_SPEED_TOP);
                CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(EJECT_FRONT_CARGO_SHIP_SPEED_BOTTOM);
            } else {
                CargoGrabberSubsystem.getInstance().setTopIntakeSpeed(EJECT_SPEED_TOP);
                CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(EJECT_SPEED_BOTTOM);
            }

            timer.reset();
        } else if (CargoArmSubsystem.getInstance().isWithinTargetAngleRange(CargoArmSubsystem.BOTTOM_ANGLE)) {
            CargoGrabberSubsystem.getInstance().setTopIntakeSpeed(INTAKE_SPEED_TOP);
            CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(INTAKE_SPEED_BOTTOM);
        } else if (CargoGrabberSubsystem.getInstance().hasLeftCargo() ||
                CargoGrabberSubsystem.getInstance().hasRightCargo()) {
            if (!CargoArmSubsystem.getInstance().isWithinTargetAngleRange(CargoArmSubsystem.getInstance().getTargetAngle())) {
                CargoGrabberSubsystem.getInstance().setTopIntakeSpeed(INTAKE_SPEED_TOP);
                CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(INTAKE_SPEED_BOTTOM);
            } else if (CargoGrabberSubsystem.getInstance().hasCargo()) {
                CargoGrabberSubsystem.getInstance().setTopIntakeSpeed(SOFT_INTAKE_SPEED_TOP);
                CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(SOFT_INTAKE_SPEED_BOTTOM);
            } else  {
                CargoGrabberSubsystem.getInstance().setTopIntakeSpeed(CENTERING_INTAKE_SPEED_TOP);
                CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(CENTERING_INTAKE_SPEED_BOTTOM);
            }
        } else if (Robot.getOi().secondaryController.getLeftTriggerAxis().get() > 0.1) {
            CargoGrabberSubsystem.getInstance().setTopIntakeSpeed(INTAKE_SPEED_TOP);
            CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(INTAKE_SPEED_BOTTOM);
        } else if (timer.get() < INTAKE_AFTER_EJECT_TIME) {
            CargoGrabberSubsystem.getInstance().setTopIntakeSpeed(INTAKE_AFTER_EJECT_SPEED_TOP);
            CargoGrabberSubsystem.getInstance().setBottomIntakeSpeed(INTAKE_AFTER_EJECT_SPEED_BOTTOM);
        } else {
            CargoGrabberSubsystem.getInstance().setIntakeSpeed(0.0);
        }
    }

    @Override
    protected void end() {
        timer.stop();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
