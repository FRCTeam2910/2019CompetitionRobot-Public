package org.frcteam2910.c2019.vision;

import org.frcteam2910.c2019.vision.api.ConfigCoprocessorPacket;
import org.frcteam2910.c2019.vision.api.TrajectoryRequestPacket;
import org.frcteam2910.c2019.vision.api.VisionClient;
import org.frcteam2910.c2019.vision.api.VisionServer;
import org.frcteam2910.common.Logger;

import java.io.IOException;

public class ControlHandler implements Runnable {
    private static final Logger LOGGER = new Logger(ControlHandler.class);

    private VisionServer<Object, Object> server;

    public ControlHandler() throws IOException {
        this.server = new VisionServer<>(5800);
    }

    @Override
    public void run() {
        while (!Thread.interrupted() && server.getSocket().isBound()) {
            try (VisionClient<Object, Object> connection = server.accept()) {
                while (!Thread.interrupted() && connection.getSocket().isConnected()) {
                    Object packet = connection.receivePacket();

                    if (packet instanceof ConfigCoprocessorPacket) {
                        // TODO: Config coprocessor
                    } else if (packet instanceof TrajectoryRequestPacket) {
                        // TODO: Generate trajectory
                    } else {
                        LOGGER.warn("Got packet with unknown type: %s", packet.getClass().getName());
                    }
                }
            } catch (IOException e) {
                LOGGER.error(e);
            }
        }
    }
}
