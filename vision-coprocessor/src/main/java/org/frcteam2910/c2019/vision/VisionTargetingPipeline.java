package org.frcteam2910.c2019.vision;

import edu.wpi.first.networktables.NetworkTable;
import org.frcteam2910.c2019.vision.drivers.Limelight;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.opencv.core.*;

import java.util.ArrayList;
import java.util.Collections;

import static org.opencv.calib3d.Calib3d.Rodrigues;
import static org.opencv.calib3d.Calib3d.solvePnP;
import static org.opencv.core.Core.PCACompute2;

public class VisionTargetingPipeline {
    private MatOfPoint3f objectPoints;
    private int numberOfPoints;
    private MatOfDouble distortionCoefficients;
    private Mat cameraMatrix;

    private Point horizontal = new Point(1, 0);
    private Limelight limelight;

    /**
     * Constructor for the pipeline
     * @param objp Object points for the target
     * @param dist Distortion Coefficients for the camera
     * @param mtx Camera Matrix for the camera
     */
    public VisionTargetingPipeline(Limelight limelight, double[][] objp, double[] dist, double[][] mtx) {
        this.limelight = limelight;

        objectPoints = doubleArrayToMatOfPoint3f(objp);
        numberOfPoints = objp.length;
        distortionCoefficients = new MatOfDouble(dist);
        cameraMatrix = doubleArrayToMat(mtx);
    }

    /**
     * Finds how far away the target is and its rotation with respect to the robot
     * @return Displacement left/right and forward/back as a Vector2
     * @throws Exception when there is no target
     */
    public RigidTransform2 getTranslation() throws Exception {
        double[][] points = new double[numberOfPoints][2];
        limelight.getCorners(numberOfPoints, points);
        if (limelight.hasTarget()) {
            Mat corners = doubleArrayToMat(points);

            // Get our principal axes
            Mat midPoint = new Mat();
            Mat x = new Mat(1, 2, CvType.CV_32F);
            Mat y = new Mat(1, 2, CvType.CV_32F);
            double skew = getPrincipalAxes(corners, midPoint, x, y);

            // Sort the image points
            sortImgPts(corners, skew, midPoint);

            // solvePnP, get our distance and angle to target
            Mat rotationVectors = new Mat();
            Mat translationVectors = new Mat();
            solvePnP(objectPoints, matToMatOfPoint2f(corners), cameraMatrix, distortionCoefficients, rotationVectors, translationVectors);
            skew = getAngleToTarget(rotationVectors);

            // Prepare and return the RigidTransformation2
            Vector2 translation = new Vector2(translationVectors.get(0, 0)[0], translationVectors.get(2, 0)[0]);
            Rotation2 rotation = Rotation2.fromRadians(skew);
            return new RigidTransform2(translation, rotation);
        }
        throw new Exception("Target not available");
    }

    /**
     * Gets the principle axes and midpoint of the given data points
     * @param corners The image points obtained from the target
     * @param mean The origin of the target in pixel coordinates
     * @param x The right principal axis
     * @param y The up principal axis
     * @return Returns the angle the principle axes are rotated at
     */
    private double getPrincipalAxes(Mat corners, Mat mean, Mat x, Mat y) {
        Mat eigenVectors = new Mat();
        Mat eigenValues = new Mat();
        PCACompute2(corners, mean, eigenVectors, eigenValues);
        double[] right = {eigenVectors.get(0, 0)[0], eigenVectors.get(1, 0)[0]};
        x.put(0, 0, right);
        double[] up = {eigenVectors.get(0, 1)[0], eigenVectors.get(1, 1)[0]};
        y.put(0, 0, up);
        return getAngle(horizontal, matToPoint(x), false);
    }

    /**
     * Sorts the given image points. Primarily sorts by angle from least to greatest, but will sort by comparing the x components of the points if the angles are too close.
     * @param corners The image points, an input/output parameter
     * @param rotation The angle that the target is rotated at. @see getPrincipalAxes()
     * @param midPoint The origin of the target in pixel coordinates
     */
    private void sortImgPts(Mat corners, double rotation, Mat midPoint) {
        double[][] imgpts = matToDoubleArray(corners);
        ArrayList<ImagePoint> points = new ArrayList<ImagePoint>();
        for (int i = 0; i < numberOfPoints; i++) {
            // Prepare the point
            Point temp = new Point(imgpts[i][0] - midPoint.get(0, 0)[0],  midPoint.get(0, 1)[0] - imgpts[i][1]);
            rotatePoint(temp, -1 * rotation);
            norm(temp);

            // Get the angle
            double angle = getAngle(horizontal, temp, true);
            ImagePoint2019 pt = new ImagePoint2019(angle, new Point(imgpts[i][0], imgpts[i][1]));
            points.add(pt);
        }
        // Now sort the arrays
        Collections.sort(points);

        // Lastly put modify the original corners matrix
        for (int i = 0; i < numberOfPoints; i++) {
            corners.put(i, 0, pointToDoubleArray(points.get(i).getPoint()));
        }
    }

    /**
     * This returns the angle to the target from the rotation vectors which was returned by solvePnP()
     * @param rotationVectors The rotation vectors calculated from solvePnP()
     * @return returns our angle to the target
     */
    private double getAngleToTarget(Mat rotationVectors) {
        Mat dst = new Mat();
        Rodrigues(rotationVectors, dst);
        dst = dst.row(2);
        Point pt = new Point(dst.get(0, 0)[0], dst.get(0, 2)[0]);
        norm(pt);
        double angle =  Math.PI - Math.acos(pt.dot(new Point(0, 1)));

        Point3 crossProduct = new Point3();
        crossProduct(new Point3(0, 0, 1), matToPoint3(dst), crossProduct);
        if (crossProduct.y < 0.0) {
            angle *= -1;
        }
        return angle;
    }

    /**
     * Gets the angle between two vectors
     * @param a The first vector
     * @param b The second vector
     * @param mode Switches between returning a angle from +/- 0-180 degrees or from 0-360 degrees. This only works given the right principal axis of the set of points this vector came from is is (1, 0). In other words, this will only work if Vector a is (1, 0).
     * @return Returns the angle between vector a and b
     */
    private static double getAngle(Point a, Point b, boolean mode) {
        double rotation = Math.acos(a.dot(b) / getLength(a) * getLength(b));
        if (b.y < 0) {
            if (mode) {
                rotation = (2 * Math.PI) - rotation;
            } else {
                rotation *= -1;
            }
        }
        return rotation;
    }

    /**
     * Rotates a point around (0, 0)
     * @param point The point to rotate
     * @param angle The angle to rotate it
     */
    private static void rotatePoint(Point point, double angle) {
        double[] result = new double[2];
        result[0] = (point.x * Math.cos(angle)) - (point.y * Math.sin(angle));
        result[1] = (point.y * Math.cos(angle)) + (point.x * Math.sin(angle));
        point.set(result);
    }

    /**
     * Returns the length of a vector
     * @param pt The vector to get the length of
     * @return The length of the vector as a double
     */
    private static double getLength(Point pt) {
        return Math.sqrt(Math.pow(pt.x, 2) + Math.pow(pt.y, 2));
    }

    /**
     * Normalizes a vector
     * @param pt The vector to normalize
     */
    private static void norm(Point pt) {
        double length = getLength(pt);
        pt.x/=length;
        pt.y/=length;
    }

    /**
     * Returns the cross product of two vectors. Remember the resulting vector differs depending on the order of the operation
     * @param a Vector A
     * @param b Vector B
     * @param result A vector where the result will be stored
     */
    private static void crossProduct(Point3 a, Point3 b, Point3 result) {
        result.x = a.y * b.z - a.z * b.y;
        result.y = a.z * b.x - a.x * b.z;
        result.z = a.x * b.y - a.y * b.x;
    }

    /**
     * Converts a matrix to a point. The matrix, of course, must represent a 2D point
     * @param mat The matrix to convert
     * @return The Point representation of the Matrix
     */
    private static Point matToPoint(Mat mat) {
        Point result = new Point();
        result.x = mat.get(0, 0)[0];
        result.y = mat.get(0, 1)[0];
        return result;
    }

    /**
     * Converts a matrix to a point. The matrix, of course, must represent a 3D point
     * @param mat The matrix to convert
     * @return The Point3 representation of the matrix
     */
    private static Point3 matToPoint3(Mat mat) {
        Point3 result = new Point3();
        result.x = mat.get(0, 0)[0];
        result.y = mat.get(0, 1)[0];
        result.z = mat.get(0, 2)[0];
        return result;
    }

    /**
     * Converts a 2 dimensional double array to a matrix
     * @param data The 2 dimensional double array to convert
     * @return The Matrix representation of the 2 dimensional array
     */
    private static Mat doubleArrayToMat(double[][] data) {
        Mat result = new Mat(data.length, data[0].length, CvType.CV_32F);
        for (int x = 0; x < data.length; x++) {
            result.put(x, 0, data[x]);
        }
        return result;
    }

    /**
     * Converts a matrix to a 2 dimensional double array
     * @param data The matrix to convert
     * @return The 2 dimensional double array of the Matrix
     */
    private static double[][] matToDoubleArray(Mat data) {
        double[][] result = new double[(int)data.size().height][2];
        for (int i = 0; i < data.size().height; i++) {
            result[i][0] = data.get(i, 0)[0];
            result[i][1] = data.get(i, 1)[0];
        }
        return result;
    }

    /**
     * Converts a 2D point to a double array
     * @param pt The point to convert
     * @return The double array representation of the point
     */
    private static double[] pointToDoubleArray(Point pt) {
        double[] result = new double[2];
        result[0] = pt.x;
        result[1] = pt.y;
        return result;
    }

    /**
     * Converts a matrix to a Matrix of 2D Points
     * @param mat The matrix to convert
     * @return The Matrix of 2D points
     */
    private static MatOfPoint2f matToMatOfPoint2f(Mat mat) {
        ArrayList<Point> points = new ArrayList<Point>();
        for (int i = 0; i < mat.size().height; i++) {
            points.add(new Point(mat.get(i, 0)[0], mat.get(i, 1)[0]));
        }
        return new MatOfPoint2f(points.toArray(new Point[0]));
    }

    // New undocumented things below

    /**
     * Converts a 2 dimensional double array to a MatOfPoint2f
     * @param corners The 2 dimensional double array to be converted
     * @return returns a MatOfPoint2f
     */
    private static MatOfPoint2f doubleArrayToMatOfPoint2f(double[][] corners) {
        ArrayList<Point> points = new ArrayList<Point>();
        for (int i = 0; i < 8; i++) {
            points.add(new Point(corners[i][0], corners[i][1]));
        }
        return new MatOfPoint2f(points.toArray(new Point[0]));
    }

    /**
     * Converts a 2 dimensional double array into a MatOfPoint3f
     * @param corners The 2 dimensional double array to be converted
     * @return returns a MatOfPoint3f
     */
    private static MatOfPoint3f doubleArrayToMatOfPoint3f(double[][] corners) {
        ArrayList<Point3> points = new ArrayList<Point3>();
        for (int i = 0; i < 8; i++) {
            points.add(new Point3(corners[i][0], corners[i][1], 0));
        }
        return new MatOfPoint3f(points.toArray(new Point3[0]));
    }
}