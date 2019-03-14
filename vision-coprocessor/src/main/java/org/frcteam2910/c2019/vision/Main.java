package org.frcteam2910.c2019.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpiutil.RuntimeLoader;
import org.opencv.core.Core;

import java.io.IOException;

public class Main {
    public static void main(String[] args) {
        try {
            RuntimeLoader<Core> loader = new RuntimeLoader<>(Core.NATIVE_LIBRARY_NAME,
                    RuntimeLoader.getDefaultExtractionRoot(), Core.class);
            loader.loadLibraryHashed();
        } catch (IOException e) {
            e.printStackTrace();
        }

        // Initialize network tables
        NetworkTableInstance.getDefault().startClientTeam(2910);

        new VisionCalculationLoop().run();

//        try {
            // Handle periodic packets in a separate thread
//            Thread periodicThread = new Thread(new PeriodicHandler());
//            periodicThread.start();

            // Handle control packets in the main thread
//            new ControlHandler().run();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
    }
}
