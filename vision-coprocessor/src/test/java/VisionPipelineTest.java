import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpiutil.RuntimeLoader;
import org.frcteam2910.c2019.vision.VisionTargetingPipeline;
import org.frcteam2910.c2019.vision.drivers.Limelight;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.opencv.core.*;

import java.io.IOException;
import java.util.ArrayList;

public class VisionPipelineTest {
    public static void main(String[] args) {
        // Initialize Networktables
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        instance.startClientTeam(2910);
        NetworkTable table = instance.getTable("limelight-cargo");

        // Load the native library
        try {
            RuntimeLoader<Core> loader = new RuntimeLoader<>(Core.NATIVE_LIBRARY_NAME, RuntimeLoader.getDefaultExtractionRoot(), Core.class);
            loader.loadLibraryHashed();
        } catch (IOException ex) {
            ex.printStackTrace();
            System.exit(1);
        }

        // Declare our object points, camera matrix, and distortion coefficients respectively
        double[][] objp = new double[][] {
                {5.9363, 2.9128, 0.0},
                {4.0, 2.4120, 0.0},
                {-4.0, 2.4120, 0.0},
                {-5.9363, 2.9128, 0.0},
                {-7.3134, -2.4120, 0.0},
                {-5.3771, -2.9128, 0.0},
                {5.3771, -2.9128, 0.0},
                {7.3134, -2.4120, 0.0}
        };

        double[][] mtx = new double[][] {
                {773.86723397, 0.0, 431.29426601},
                {0.0, 777.62582296, 306.80364947},
                {0.0, 0.0, 1.0}
        };

        double[] dist = new double[] {3.15600348e-01, -1.17776987e+00, -9.30063427e-03, 1.46275541e-03, 1.61055001e+00};

        // Create an instance of our vision targeting pipeline
        VisionTargetingPipeline pipeline = new VisionTargetingPipeline(new Limelight(table), objp, dist, mtx);
        RigidTransform2 transform;

        while (true) {
            try {
                transform = pipeline.getTranslation();
                System.out.println(transform.toString());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}