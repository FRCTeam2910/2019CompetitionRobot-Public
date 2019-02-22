package org.frcteam2910.c2019.vision.api;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.junit.Test;

import java.io.IOException;
import java.net.InetAddress;

import static org.junit.Assert.assertEquals;

public class VisionClientServerTest {
    @Test
    public void sendReceiveTest() {
        PeriodicPacket expectedPacket = new PeriodicPacket(
                0.0,
                new RigidTransform2(new Vector2(1.0, 0.0), Rotation2.ZERO),
                new Vector2(-5.0, 0.2),
                0.2
        );
        PeriodicPacket actualPacket;

        try (VisionServer<Object, PeriodicPacket> server = new VisionServer<>(8888)) {
            Thread clientThread = new Thread(() -> {
                try (VisionClient<PeriodicPacket, Object> client = new VisionClient<>(InetAddress.getLocalHost(), 8888)) {
                    client.sendPacket(expectedPacket);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            });
            clientThread.start();

            VisionClient<Object, PeriodicPacket> connection = server.accept();

            actualPacket = connection.receivePacket();

            assertEquals("Timestamp does not match", expectedPacket.getTimestamp(),
                    actualPacket.getTimestamp(), MathUtils.EPSILON);
            assertEquals("Pose does not match", expectedPacket.getPose(), actualPacket.getPose());
            assertEquals("Velocity does not match", expectedPacket.getVelocity(), actualPacket.getVelocity());
            assertEquals("Rotational velocity does not match", expectedPacket.getRotationalVelocity(),
                    actualPacket.getRotationalVelocity(), MathUtils.EPSILON);

            clientThread.join();
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }
}
