package org.firstinspires.ftc.teamcode.lupinePather.pathGeneration;

public class Vector {
    /**
     * IMPORTANT NOTE: theta is defined in radians
     */
    // these are the values of the coordinate defined by this Point, in both polar and Cartesian systems
    private double magnitude, theta, xComponent, yComponent;


    /**
     * This creates a new Point with a set coordinate system
     *
     * @param magnitude magnitude of the vector
     * @param theta the direction of the vector in radians
     */
    public Vector(double magnitude, double theta) {
        setComponents(magnitude, theta);
    }

    /**
     * This simply sets the polar components of the vector
     *
     * @param magnitude sets the magnitude of this vector
     * @param theta sets the theta value of this vector
     */
    public void setComponents(double magnitude, double theta) {
        double[] orthogonalComponents;
        this.magnitude = magnitude;
        this.theta = theta;
        orthogonalComponents = Point.polarToCartesian(magnitude, theta);
        xComponent = orthogonalComponents[0];
        yComponent = orthogonalComponents[1];
    }

    /**
     * This sets only the magnitude of the vector
     *
     * @param magnitude sets the magnitude of this vector
     */
    public void setMagnitude(double magnitude) {
        setComponents(magnitude, theta);
    }

    /**
     * This sets only the angle theta of the vector
     *
     * @param theta sets the magnitude of this vector
     */
    public void setTheta(double theta) {
        setComponents(magnitude, theta);
    }

    /**
     * This simply sets the orthogonal components of the vector
     *
     * @param xComponent sets the x component of this vector
     * @param yComponent sets the y component of this vector
     */
    public void setOrthogonalComponents(double xComponent, double yComponent) {
        double[] polarComponents;
        this.xComponent = xComponent;
        this.yComponent = yComponent;
        polarComponents = Point.cartesianToPolar(xComponent, yComponent);
        magnitude = polarComponents[0];
        theta = polarComponents[1];
    }

    /**
     * Returns the magnitude of this vector
     *
     * @return returns the magnitude
     */
    public double getMagnitude() {
        return magnitude;
    }

    /**
     * Returns the theta value of this vector
     *
     * @return returns the theta value
     */
    public double getTheta() {
        return theta;
    }

    /**
     * Returns the x component of this vector
     *
     * @return returns the x component
     */
    public double getXComponent() {
        return xComponent;
    }

    /**
     * Returns the y component of this vector
     *
     * @return returns the y component
     */
    public double getYComponent() {
        return yComponent;
    }
}