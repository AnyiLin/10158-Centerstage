package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Point {
    /**
     * IMPORTANT NOTE: theta is defined in radians
     */
    // these are the values of the coordinate defined by this Point, in both polar and Cartesian systems
    private double r, theta, x, y;

    // these are used for ease of changing/setting identification
    public static final int POLAR = 0, CARTESIAN = 1;


    /**
     * This creates a new Point with a set coordinate system
     *
     * @param rOrX depending on the coordinate system specified, this is either the r or x value
     * @param thetaOrY depending on the coordinate system specified, this is either the theta or y value
     * @param identifier this specifies what coordinate system this Point will initially use
     */
    public Point(double rOrX, double thetaOrY, int identifier) {
        setCoordinates(rOrX, thetaOrY, identifier);
    }

    /**
     * This creates a new Point from a Pose2d
     *
     * @param pose the pose
     */
    public Point(Pose2d pose) {
        setCoordinates(pose.getX(), pose.getY(), CARTESIAN);
    }

    /**
     * This simply sets the coordinates of the Point using the existing coordinate system
     *
     * @param rOrX depending on the current coordinate system, this is either the r or x value
     * @param thetaOrY depending on the current coordinate system, this is either the theta or y value
     * @param identifier this specifies what coordinates to use when setting values
     */
    public void setCoordinates(double rOrX, double thetaOrY, int identifier) {
        double[] setOtherCoordinates;
        switch (identifier) { // this detects which coordinate system to use
            // there is no POLAR case since that's covered by the default
            case CARTESIAN:
                x = rOrX;
                y = thetaOrY;
                setOtherCoordinates = cartesianToPolar(x, y);
                r = setOtherCoordinates[0];
                theta = setOtherCoordinates[1];
                break;
            default:
                if (rOrX<0) {
                    r = -rOrX;
                    theta = MathFunctions.normalizeAngle(thetaOrY+Math.PI);
                } else {
                    r = rOrX;
                    theta = MathFunctions.normalizeAngle(thetaOrY);
                }
                setOtherCoordinates = polarToCartesian(r, theta);
                x = setOtherCoordinates[0];
                y = setOtherCoordinates[1];
                break;
        }
    }

    /**
     * Calculates the distance between this point and some other specified point
     *
     * @param otherPoint the other specified point
     * @return returns the distance between the two
     */
    public double distanceFrom(Point otherPoint) {
        return Math.sqrt(Math.pow(otherPoint.getX()-x, 2) + Math.pow(otherPoint.getY()-y, 2));
    }

    /**
     * This takes in an r and theta value and converts them to Cartesian coordinates
     *
     * @param r this is the r value of the point being converted
     * @param theta this is the theta value of the point being converted
     * @return this returns an array of doubles of length 2 with the x and y values in that order in it
     */
    public static double[] polarToCartesian(double r, double theta) {
        return new double[] {r * Math.cos(theta), r * Math.sin(theta)};
    }

    /**
     * This takes in an x and y value and converts them to polar coordinates
     *
     * @param x this is the x value of the point being converted
     * @param y this is the y value of the point being converted
     * @return this returns an array of doubles of length 2 with the r and theta values in that order in it
     */
    public static double[] cartesianToPolar(double x, double y) {
        if (x == 0) {
            if (y > 0) {
                return new double[] {Math.abs(y), Math.PI/2};
            } else {
                return new double[] {Math.abs(y), (3 * Math.PI) / 2};
            }
        }
        double r = Math.sqrt(x*x+y*y);
        if (x < 0) return new double[] {r, Math.PI+Math.atan(y/x)};
        if (y > 0) {
            return new double[]{r, Math.atan(y / x)};
        } else {
            return new double[]{r, (2*Math.PI) + Math.atan(y / x)};
        }
    }

    /**
     * Returns the r value of this Point
     *
     * @return returns the r value
     */
    public double getR() {
        return r;
    }

    /**
     * Returns the theta value of this Point
     *
     * @return returns the theta value
     */
    public double getTheta() {
        return theta;
    }

    /**
     * Returns the x value of this Point
     *
     * @return returns the x value
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y value of this Point
     *
     * @return returns the y value
     */
    public double getY() {
        return y;
    }
}