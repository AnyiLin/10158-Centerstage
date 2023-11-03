package org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration;

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
        Vector returnVector = new Vector(0,0);
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
        Vector returnVector = new Vector(0,0);
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
}
