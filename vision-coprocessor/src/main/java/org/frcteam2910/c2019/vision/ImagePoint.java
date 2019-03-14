package org.frcteam2910.c2019.vision;

import org.opencv.core.Point;

public class ImagePoint implements Comparable<ImagePoint> {
    private double angle;
    private Point point;

    /**
     * Default constructor
     */
    public ImagePoint() {
        this(0.0, new Point(0.0, 0.0));
    }

    /**
     * Main constructor
     * @param angle The angle between this point and it's right principal axis
     * @param point The point in pixel coordinates
     */
    public ImagePoint(double angle, Point point) {
        this.angle = angle;
        this.point = point;
    }

    /**
     * Return the angle for this Image Point
     * @return The angle for this image point
     */
    public double getAngle() {
        return angle;
    }

    /**
     * Return the point in pixel coordinates for this point
     * @return The point in pixel coordinates for this image point
     */
    public Point getPoint() {
        return point;
    }

    /**
     * Set the angle for the Image Point. This is the angle between this point and it's right principal axis
     * @param angle
     */
    public void setAngle(double angle) {
        this.angle = angle;
    }

    /**
     * Set the point for this Image Point. 
     * @param point
     */
    public void setPoint(Point point) {
        this.point = point;
    }

    /**
     * Returns a string representation of this point for debugging purposes
     * @return A string representation of this point
     */
    public String toString() {
        return point.toString() + ", " + Math.toDegrees(angle);
    }

    /**
     * Compares against another point based on the angle.
     * @param other The other point to compare to
     * @return returns 1 if the other point is greater than this point, -1 if this point is less than the other point, and 0 if this point is equal to the other point
     */
    @Override
    public int compareTo(ImagePoint other) {
        if (this.angle < other.getAngle()) {
            return -1;
        }
        if (this.angle == other.getAngle()) {
            return 0;
        }
        return 1;
    }
}
