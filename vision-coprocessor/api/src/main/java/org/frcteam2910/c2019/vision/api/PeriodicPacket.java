package org.frcteam2910.c2019.vision.api;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;

import java.io.Serializable;

public class PeriodicPacket implements Serializable {
    private static final long serialVersionUID = 5090957247874698962L;

    private final double timestamp;
    private final RigidTransform2 pose;
    private final Vector2 velocity;
    private final double rotationalVelocity;

    public PeriodicPacket(double timestamp, RigidTransform2 pose, Vector2 velocity, double rotationalVelocity) {
        this.timestamp = timestamp;
        this.pose = pose;
        this.velocity = velocity;
        this.rotationalVelocity = rotationalVelocity;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public RigidTransform2 getPose() {
        return pose;
    }

    public Vector2 getVelocity() {
        return velocity;
    }

    public double getRotationalVelocity() {
        return rotationalVelocity;
    }
}
