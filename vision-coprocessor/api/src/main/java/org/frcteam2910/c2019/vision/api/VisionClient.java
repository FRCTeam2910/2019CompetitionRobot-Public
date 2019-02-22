package org.frcteam2910.c2019.vision.api;

import org.frcteam2910.common.Logger;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.InetAddress;
import java.net.Socket;

public class VisionClient<SendPacketType, ReceivePacketType> implements AutoCloseable {
    private static final Logger LOGGER = new Logger(VisionClient.class);

    private Socket socket;

    VisionClient(Socket socket) {
        this.socket = socket;
    }

    public VisionClient(String address, int port) throws IOException {
        this(InetAddress.getByName(address), port);
    }

    public VisionClient(InetAddress address, int port) throws IOException {
        this.socket = new Socket(address, port);
    }

    public void sendPacket(SendPacketType packet) throws IOException {
        ObjectOutputStream out = new ObjectOutputStream(socket.getOutputStream());

        out.writeObject(packet);
    }

    public ReceivePacketType receivePacket() throws IOException {
        ObjectInputStream in = new ObjectInputStream(socket.getInputStream());

        // Block until we receive a valid packet or our connection to the server closes.
        while (!Thread.interrupted() && socket.isConnected()) {
            try {
                return (ReceivePacketType) in.readObject();
            } catch (ClassNotFoundException e) {
                LOGGER.error(e);
            }
        }

        throw new IOException("Connection was closed.");
    }

    @Override
    public void close() throws IOException {
        socket.close();
    }

    public Socket getSocket() {
        return socket;
    }
}
