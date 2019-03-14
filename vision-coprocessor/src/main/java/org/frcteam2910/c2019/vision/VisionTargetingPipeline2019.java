package org.frcteam2910.c2019.vision;

import org.frcteam2910.c2019.vision.drivers.Limelight;

public class VisionTargetingPipeline2019 extends VisionTargetingPipeline {
    private static final double[][] OBJET_POINTS = {
            {5.9363, 2.9128, 0.0},
            {4.0, 2.4120, 0.0},
            {-4.0, 2.4120, 0.0},
            {-5.9363, 2.9128, 0.0},
            {-7.3134, -2.4120, 0.0},
            {-5.3771, -2.9128, 0.0},
            {5.3771, -2.9128, 0.0},
            {7.3134, -2.4120, 0.0}
    };

    private static final double[][] CAMERA_MATRIX = {
            {773.86723397, 0.0, 431.29426601},
            {0.0, 777.62582296, 306.80364947},
            {0.0, 0.0, 1.0}
    };

    private static final double[] DISTORTION_COEFFICIENTS = {
            3.15600348e-01, -1.17776987e+00, -9.30063427e-03, 1.46275541e-03, 1.61055001e+00
    };


    public VisionTargetingPipeline2019(Limelight limelight) {
        super(limelight, OBJET_POINTS, DISTORTION_COEFFICIENTS, CAMERA_MATRIX);
    }
}
