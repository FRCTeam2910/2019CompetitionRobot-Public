package org.frcteam2910.c2019.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.frcteam2910.c2019.vision.api.*;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.subsystems.Subsystem;

import java.io.IOException;

public class VisionCoprocessorSubsystem extends Subsystem {
    private static final Logger LOGGER = new Logger(VisionCoprocessorSubsystem.class);

    private static final String COPROCESSOR_HOSTNAME = "raspberrypi.local";

    /**
     * How much time to wait between connection attempts for the control connection
     */
    private static final double CONTROL_RECONNECT_WAIT_TIME = 0.5;

    /**
     * How often to send a periodic update in hertz
     */
    private static final double PERIODIC_UPDATE_RATE = 200.0;
    /**
     * How much time to wait between connection attempts for the periodic connection
     */
    private static final double PERIODIC_RECONNECT_WAIT_TIME = 1.0;

    private static final VisionCoprocessorSubsystem instance = new VisionCoprocessorSubsystem();

    private final Object controlConnectionLock = new Object();
    private final Object controlConnectionNotifier = new Object();
    private VisionClient<Object, Object> controlConnection = null;

    private Notifier periodicUpdateNotifier = new Notifier(new Runnable() {
        private VisionClient<PeriodicPacket, Object> client = null;

        @Override
        public void run() {
            if (client == null) {
                try {
                    client = new VisionClient<>(COPROCESSOR_HOSTNAME, Ports.PERIODIC_PORT);
                } catch (IOException e) {
                    // We failed to connect. Wait a little bit and try again.
                    LOGGER.warn(e);
                    Timer.delay(PERIODIC_RECONNECT_WAIT_TIME);
                    return;
                }
            }

            double timestamp = Timer.getFPGATimestamp();
            RigidTransform2 pose = new RigidTransform2(
                    DrivetrainSubsystem.getInstance().getKinematicPosition(),
                    Superstructure.getInstance().getGyroscope().getAngle()
            );
            Vector2 velocity = DrivetrainSubsystem.getInstance().getKinematicVelocity();
            double rotationalVelocity = Superstructure.getInstance().getGyroscope().getRate();

            PeriodicPacket packet = new PeriodicPacket(timestamp, pose, velocity, rotationalVelocity);

            try {
                client.sendPacket(packet);
            } catch (IOException e) {
                try {
                    client.close();
                } catch (IOException e1) {
                    e.addSuppressed(e1);
                }
                client = null;
                LOGGER.error(e);
            }
        }
    });

    public VisionCoprocessorSubsystem() {
        periodicUpdateNotifier.startPeriodic(1.0 / PERIODIC_UPDATE_RATE);

        // Start a new thread which tries to connect to the coprocessor's control port
        new Thread(() -> {
            while (!Thread.interrupted()) {
                // We are the only thread which writes to the controlConnection. Check if there is a valid connection.
                boolean hasValidConnection;
                synchronized (controlConnectionLock) {
                    if (controlConnection == null) {
                        hasValidConnection = false;
                    } else {
                        hasValidConnection = controlConnection.getSocket().isConnected();
                    }

                    // If there is no valid connection, set it to null so the other threads know
                    controlConnection = null;
                }

                // If we don't have a valid connection, attempt to connect to the coprocessor, otherwise, wait to be
                // notified about an error.
                if (!hasValidConnection) {
                    try {
                        VisionClient<Object, Object> connection = new VisionClient<>(COPROCESSOR_HOSTNAME,
                                Ports.CONTROL_PORT);
                        // We were able to connect. Set the control connection
                        synchronized (controlConnectionLock) {
                            controlConnection = connection;
                        }
                    } catch (IOException e) {
                        LOGGER.warn(e);

                        // Wait a little before trying to reconnect again
                        Timer.delay(CONTROL_RECONNECT_WAIT_TIME);
                    }
                } else {
                    synchronized (controlConnectionNotifier) {
                        try {
                            controlConnectionNotifier.wait(500);
                        } catch (InterruptedException ignored) {
                        }
                    }
                }
            }
        });
    }

    public static VisionCoprocessorSubsystem getInstance() {
        return instance;
    }

    private void reconnectToCoprocessor() throws IOException {
        synchronized (controlConnectionLock) {
            // Close the existing connection to the coprocessor
            controlConnection.close();
        }

        // Wake up the connector thread to reconnect asynchronously
        controlConnectionNotifier.notify();
    }

    private void sendControlPacket(Object packet) throws IOException {
        try {
            synchronized (controlConnectionLock) {
                // The control connection can be null if there is no active connection
                if (controlConnection == null) {
                    throw new IOException("Not currently connected to the coprocessor");
                }

                controlConnection.sendPacket(packet);
            }
        } catch (IOException e) {
            // We were not able to communicate with the coprocessor. Reconnect to the coprocessor
            reconnectToCoprocessor();

            throw new IOException("Unable to send control packet to the coprocessor", e);
        }
    }

    private Object receiveControlResponse() throws IOException {
        try {
            synchronized (controlConnectionLock) {
                // The control connection can be null if there is no active connection
                if (controlConnection == null) {
                    throw new IOException("Not currently connected to the coprocessor");
                }

                return controlConnection.receivePacket();
            }
        } catch (IOException e) {
            // We were not able to communicate with the coprocessor. Reconnect to the coprocessor
            reconnectToCoprocessor();

            throw new IOException("Unable to receive control response from the coprocessor", e);
        }
    }

    public Trajectory requestTrajectory(Gamepiece gamepiece) throws IOException {
        TrajectoryRequestPacket packet = new TrajectoryRequestPacket(Timer.getFPGATimestamp(), gamepiece);

        sendControlPacket(packet);

        // Wait for the result to come back from the coprocessor
        Object response = receiveControlResponse();

        try {
            return (Trajectory) response;
        } catch (ClassCastException e) {
            // We didn't get the correct response type back
            throw new IOException(String.format("Got an invalid response type from coprocessor. (Expected %s, got %s)",
                    Trajectory.class.getName(), response.getClass().getName()), e);
        }
    }

    /**
     * Sets the driver mode for the specified camera.
     *
     * @param gamepiece  which gamepiece camera to set driver mode for
     * @param driverMode if driver mode should be enabled or disabled
     * @throws IOException if we failed to communicate with the coprocessor
     */
    public void setDriverMode(Gamepiece gamepiece, boolean driverMode) throws IOException {
        ConfigCoprocessorPacket packet = new ConfigCoprocessorPacket(gamepiece, driverMode);

        sendControlPacket(packet);
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

    }
}
