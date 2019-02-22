package org.frcteam2910.c2019.vision.api;

import java.io.Serializable;

public class TrajectoryRequestPacket implements Serializable {
    private static final long serialVersionUID = 6087006825146565015L;

    private final double timestamp;
    private final Gamepiece gamepiece;

    public TrajectoryRequestPacket(double timestamp, Gamepiece gamepiece) {
        this.timestamp = timestamp;
        this.gamepiece = gamepiece;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public Gamepiece getGamepiece() {
        return gamepiece;
    }
}
