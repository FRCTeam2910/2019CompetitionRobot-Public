import edu.wpi.first.wpiutil.RuntimeLoader;
import org.opencv.core.*;
import java.io.IOException;

public class Test {
    public static void main(String[] args) {
        try {
            RuntimeLoader<Core> loader = new RuntimeLoader<>(Core.NATIVE_LIBRARY_NAME, RuntimeLoader.getDefaultExtractionRoot(), Core.class);
            loader.loadLibraryHashed();
        } catch (IOException ex) {
            ex.printStackTrace();
            System.exit(1);
        }

        Point point1 = new Point(1, 2);
        Point point2 = new Point(3, 4);
        System.out.println(point1.dot(point2));
    }
}