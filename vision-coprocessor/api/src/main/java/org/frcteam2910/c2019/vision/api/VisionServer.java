package org.frcteam2910.c2019.vision.api;

import java.io.IOException;
import java.net.ServerSocket;

public class VisionServer<SendPacketType, ReceivePacketType> implements AutoCloseable {
    private ServerSocket socket;

    public VisionServer(int port) throws IOException {
        this.socket = new ServerSocket(port);
    }

    public VisionClient<SendPacketType, ReceivePacketType> accept() throws IOException {
        return new VisionClient<>(socket.accept());
    }

    @Override
    public void close() throws IOException {
        socket.close();
    }

    public ServerSocket getSocket() {
        return socket;
    }
}
