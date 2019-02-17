package org.frcteam2910.c2019.subsystems;

import edu.wpi.first.wpilibj.SPI;
import org.frcteam2910.common.robot.drivers.NavX;

public class Superstructure {
    private static final Superstructure instance = new Superstructure();

    private NavX navX = new NavX(SPI.Port.kMXP);

    private Superstructure() {
        navX.calibrate();
    }

    public static Superstructure getInstance() {
        return instance;
    }

    public NavX getGyroscope() {
        return navX;
    }
}
