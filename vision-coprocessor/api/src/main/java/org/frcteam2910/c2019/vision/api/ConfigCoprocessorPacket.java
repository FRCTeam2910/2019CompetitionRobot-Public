package org.frcteam2910.c2019.vision.api;

import java.io.Serializable;

public class ConfigCoprocessorPacket implements Serializable {
    private final Gamepiece gamepiece;
    private final boolean driverMode;

    public ConfigCoprocessorPacket(Gamepiece gamepiece, boolean driverMode) {
        this.gamepiece = gamepiece;
        this.driverMode = driverMode;
    }

    public Gamepiece getGamepiece() {
        return gamepiece;
    }

    public boolean isDriverMode() {
        return driverMode;
    }
}
