package org.frcteam2910.c2019.vision;

import java.io.IOException;

public class Main {
    public static void main(String[] args) {
        try {
            // Handle periodic packets in a separate thread
            Thread periodicThread = new Thread(new PeriodicHandler());
            periodicThread.start();

            // Handle control packets in the main thread
            new ControlHandler().run();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
