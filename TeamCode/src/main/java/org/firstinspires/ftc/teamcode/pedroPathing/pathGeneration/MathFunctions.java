package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MathFunctions {
    /**
     * There is possibly a more efficient method of doing this, but I'm doing it this way
     * because it's simple and doesn't have many rounding errors in the middle
     *
     * @param n this is how many you want to choose from
     * @param r this is how many you want to choose
     * @return returns the result of the "n choose r" function
     */
    public static double nCr(int n, int r) {
        double num = 1;
        double denom = 1;

        // this multiplies up the numerator of the nCr function
        for (int i = n; i > n-r; i--) {
            num *= i;
        }

        // this multiplies up the denominator of the nCr function
        for (int i = 1; i <=r; i++) {
            denom *= i;
        }

        return num/denom;
    }

    /**
     * This returns the sign of a number
     *
     * @param get the number
     * @return returns the sign of the number
     */
    public static double getSign(double get) {
        if (get == 0) return 0;
        if (get > 0) return 1;
        return -1;
    }

    /**
     * This clamps down a value to between the lower and upper bounds
     *
     * @param num the number to be clamped
     * @param lower the lower bound
     * @param upper the upper bound
     * @return returns the clamped number
     */
    public static double clamp(double num, double lower, double upper) {
        if (num < lower) return lower;
        if (num > upper) return upper;
        return num;
    }

    /**
     * This normalizes an angle to be between 0 and 2pi radians, inclusive
     *
     * IMPORTANT NOTE: This method operates in radians
     *
     * @param angleRadians the angle to be normalized
     * @return returns the normalized angle
     */
    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians;
        while (angle<0) angle += 2*Math.PI;
        while (angle>2*Math.PI) angle -= 2*Math.PI;
        return angle;
    }

    /**
     * This returns the smallest angle between two angles
     *
     * @param one one of the angles
     * @param two the other one
     * @return returns the smallest angle
     */
    public static double getSmallestAngleDifference(double one, double two) {
        return Math.min(MathFunctions.normalizeAngle(one-two), MathFunctions.normalizeAngle(two-one));
    }

    /**
     * This gets the direction to turn between a start heading and an end heading
     *
     * @return returns the turn direction
     */
    public static double getTurnDirection(double startHeading, double endHeading) {
        if (MathFunctions.normalizeAngle(endHeading-startHeading) >= 0 && MathFunctions.normalizeAngle(endHeading-startHeading) <= Math.PI) {
            return 1; // counter clock wise
        }
        return -1; // clock wise
    }

    /**
     * This returns the distance between a Pose2d and a point
     *
     * @param pose this is the pose
     * @param point this is the point
     * @return returns the distance between the two
     */
    public static double distance(Pose2d pose, Point point) {
        return Math.sqrt(Math.pow(pose.getX()-point.getX(), 2) + Math.pow(pose.getY()-point.getY(), 2));
    }

    /**
     * This returns the distance between a Pose2d and another Pose2d
     *
     * @param one this is the first pose
     * @param two this is the second pose
     * @return returns the distance between the two
     */
    public static double distance(Pose2d one, Pose2d two) {
        return Math.sqrt(Math.pow(one.getX()-two.getX(), 2) + Math.pow(one.getY()-two.getY(), 2));
    }

    /**
     * This returns a point that is the sum of the two input points
     *
     * @param one the first point
     * @param two the second point
     * @return returns the sum of the two points
     */
    public static Point addPoints(Point one, Point two) {
        return new Point(one.getX() + two.getX(), one.getY() + two.getY(), Point.CARTESIAN);
    }

    /**
     * This subtracts the second point from the first point and returns the result as a point
     * Do note that order matters here
     *
     * @param one the first point
     * @param two the second point
     * @return returns the difference of the two points
     */
    public static Point subtractPoints(Point one, Point two) {
        return new Point(one.getX() - two.getX(), one.getY() - two.getY(), Point.CARTESIAN);
    }

    /**
     * This multiplies a point by a scalar and returns the result as a point
     *
     * @param point the point being multiplied
     * @param scalar the scalar multiplying into the point
     * @return returns the scaled point
     */
    public static Point scalarMultiplyPoint(Point point, double scalar) {
        return new Point(point.getX()*scalar, point.getY()*scalar, Point.CARTESIAN);
    }

    /**
     * Copies a Point, but with a different reference location in the memory
     *
     * @param point point to be deep copied
     * @return returns the copied point
     */
    public static Point copyPoint(Point point) {
        return new Point(point.getX(), point.getY(), Point.CARTESIAN);
    }

    /**
     * Copies a Vector, but with a different reference location in the memory
     *
     * @param vector vector to be deep copied
     * @return returns the copied vector
     */
    public static Vector copyVector(Vector vector) {
        return new Vector(vector.getMagnitude(), vector.getTheta());
    }

    /**
     * This multiplies a vector by a scalar and returns the result as a vector
     *
     * @param vector the vector being multiplied
     * @param scalar the scalar multiplying into the vector
     * @return returns the scaled vector
     */
    public static Vector scalarMultiplyVector(Vector vector, double scalar) {
        return new Vector(vector.getMagnitude()*scalar, vector.getTheta());
    }

    /**
     * This normalizes a vector to be of magnitude 1, unless the vector is the zero vector
     * In that case, it just returns back the original vector but with a different memory location
     *
     * @param vector the vector being normalized
     * @return returns the normalized (or zero) vector
     */
    public static Vector normalizeVector(Vector vector) {
        if (vector.getMagnitude() == 0) {
            return new Vector(0.0, vector.getTheta());
        } else {
            return new Vector(vector.getMagnitude()/Math.abs(vector.getMagnitude()), vector.getTheta());
        }
    }

    /**
     * This returns a vector that is the sum of the two input vectors
     *
     * @param one the first vector
     * @param two the second vector
     * @return returns the sum of the vectors
     */
    public static Vector addVectors(Vector one, Vector two) {
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(one.getXComponent()+two.getXComponent(), one.getYComponent()+two.getYComponent());
        return returnVector;
    }

    /**
     * This subtracts the second vector from the first vector and returns the result as a vector
     * Do note that order matters here
     *
     * @param one the first vector
     * @param two the second vector
     * @return returns the second vector subtracted from the first vector
     */
    public static Vector subtractVectors(Vector one, Vector two) {
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(one.getXComponent()-two.getXComponent(), one.getYComponent()-two.getYComponent());
        return returnVector;
    }

    /**
     * This computes the dot product of the two vectors
     *
     * @param one the first vector
     * @param two the second vector
     * @return returns the dot product of the two vectors
     */
    public static double dotProduct(Vector one, Vector two) {
        return one.getXComponent()*two.getXComponent() + one.getYComponent()*two.getYComponent();
    }

    /**
     * This computes the first vector crossed with the second vector
     * Do note that order matters here
     *
     * @param one the first vector
     * @param two the second vector
     * @return returns the cross product of the two vectors
     */
    public static double crossProduct(Vector one, Vector two) {
        return one.getXComponent()*two.getYComponent() - one.getYComponent()*two.getXComponent();
    }

    /**
     * This returns whether one is within two by plus/minus accuracy amount.
     *
     * @param one first number
     * @param two second number
     * @param accuracy how close is acceptable
     * @return returns if one is close enough to two
     */
    public static boolean roughlyEquals(double one, double two, double accuracy) {
        return (one < two + accuracy && one > two - accuracy);
    }

    /**
     * This returns whether one is within two by plus/minus 0.0001.
     *
     * @param one first number
     * @param two second number
     * @return returns if one is within plus/minus 0.0001 of two
     */
    public static boolean roughlyEquals(double one, double two) {
        return roughlyEquals(one, two, 0.0001);
    }
}
