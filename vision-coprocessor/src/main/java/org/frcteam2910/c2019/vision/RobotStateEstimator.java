package org.frcteam2910.c2019.vision;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.Interpolable;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

public class RobotStateEstimator {
    private static final int MAXIMUM_STATE_COUNT = 400;

    private static final Object stateLock = new Object();
    private static InterpolatingTreeMap<InterpolatingDouble, State> states = new InterpolatingTreeMap<>(MAXIMUM_STATE_COUNT);

    public static void addState(double timestamp, RigidTransform2 pose, Vector2 velocity, double rotationalVelocity) {
        InterpolatingDouble key = new InterpolatingDouble(timestamp);
        State state = new State(timestamp, pose, velocity, rotationalVelocity);

        synchronized (stateLock) {
            states.put(key, state);
        }
    }

    public static State estimateState(double timestamp) {
        InterpolatingDouble key = new InterpolatingDouble(timestamp);

        synchronized (stateLock) {
            return states.getInterpolated(key);
        }
    }

    public static class State implements Interpolable<State> {
        private final double timestamp;
        private final RigidTransform2 pose;
        private final Vector2 velocity;
        private final double rotationalVelocity;

        public State(double timestamp, RigidTransform2 pose, Vector2 velocity, double rotationalVelocity) {
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

        @Override
        public State interpolate(State other, double t) {
            return new State(
                    new InterpolatingDouble(timestamp).interpolate(new InterpolatingDouble(other.timestamp), t).value,
                    pose.interpolate(other.pose, t),
                    velocity.interpolate(other.velocity, t),
                    new InterpolatingDouble(rotationalVelocity).interpolate(new InterpolatingDouble(other.rotationalVelocity), t).value
            );
        }
    }
}
