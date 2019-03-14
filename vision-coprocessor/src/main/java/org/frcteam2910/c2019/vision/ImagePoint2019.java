package org.frcteam2910.c2019.vision;

import org.opencv.core.Point;

public class ImagePoint2019 extends ImagePoint {
    private static double MIN_ANGLE_DIFFERENCE = Math.toRadians(0.5);

    /**
     * Default constructor
     */
    public ImagePoint2019() {
        this(0.0, new Point(0.0, 0.0));
    }

    /**
     * Main constructor
     * @param angle Angle between this point and the horizontal
     * @param point The point represented in pixel coordinates
     */
    public ImagePoint2019(double angle, Point point) {
        super(angle, point);
    }

    /**
     * Compares the current point with the other. If the instance of this point is just an ImagePoint, or if the angle between this point and the other ess than 0.5 degrees, then call the super compareTo().
     * Otherwise, this means the angle of the two points are too close so we must compare them by there x components. This is done by finding which quadrant the points are in then returning either 1 or -1 depending on which point is which.
     * Thus, this compareTo() is simply a backup in case the angles are too close.
     * @param other The other point to compare to
     * @return
     */
    @Override
    public int compareTo(ImagePoint other) {
        if (!(other instanceof ImagePoint2019) || (Math.abs(other.getAngle() - this.getAngle()) > MIN_ANGLE_DIFFERENCE)) {
            return super.compareTo(other);
        }
        if ((0.0 < this.getAngle()) && (this.getAngle() < (Math.PI / 2))) {
            if (this.getPoint().x < other.getPoint().x) {
                // "this" is point 2
                return 1;
            }
            if (this.getPoint().x > other.getPoint().x) {
                // "this" is point 1
                return -1;
            }
            return 0;
        }
        if (((Math.PI / 2) < this.getAngle()) && (this.getAngle() < Math.PI)) {
            if (this.getPoint().x < other.getPoint().x) {
                // "this" is point 4
                return 1;
            }
            if (this.getPoint().x > other.getPoint().x) {
                // "this" is point 3
                return -1;
            }
            return 0;
        }
        if ((Math.PI < this.getAngle()) && (this.getAngle() < ((3 * Math.PI) / 2))) {
            if (this.getPoint().x < other.getPoint().x) {
                // "this" is point 5
                return -1;
            }
            if (this.getPoint().x > other.getPoint().x) {
                // "this" is point 6
                return 1;
            }
            return 0;
        }
        if ((((3 * Math.PI) / 2) < this.getAngle()) && (this.getAngle() < (2 * Math.PI))) {
            if (this.getPoint().x < other.getPoint().x) {
                // "this" is point 7
                return -1;
            }
            if (this.getPoint().x > other.getPoint().x) {
                // "this" is point 8
                return 1;
            }
            return 0;
        }
        return 0;
    }
}
