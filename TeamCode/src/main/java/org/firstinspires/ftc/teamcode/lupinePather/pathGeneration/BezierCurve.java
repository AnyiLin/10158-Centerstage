package org.firstinspires.ftc.teamcode.lupinePather.pathGeneration;

import java.util.ArrayList;

public class BezierCurve {
    // This contains the coefficients for the curve points
    private ArrayList<BezierCurveCoefficients> pointCoefficients = new ArrayList<>();

    // This contains the control points for the Bezier curve
    private ArrayList<Point> controlPoints;

    private final int APPROXIMATION_STEPS = 20;

    private double UNIT_TO_TIME, length;

    /**
     * This creates a new Bezier curve with an ArrayList of control points and generates the curve
     * IMPORTANT NOTE: The order of the control points is important. That's the order the code will
     * process them in, with the 0 index being the start point and the final index being the end point
     *
     * @param controlPoints This is the ArrayList of control points that define the Bezier curve
     */
    public BezierCurve(ArrayList<Point> controlPoints) {
        if (controlPoints.size()<3) {
            //TODO: throw some sort of error
        }
        this.controlPoints = controlPoints;
        generateBezierCurve();
        length = approximateLength();
        UNIT_TO_TIME = 1/length;
    }

    /**
     * This generates the Bezier curve. It assumes that the ArrayList of control points has been set.
     *
     * See https://en.wikipedia.org/wiki/Bézier_curve for the explicit formula for Bezier curves
     */
    public void generateBezierCurve() {
        int n = controlPoints.size()-1;
        for (int i = 0; i <= n; i++) {
            pointCoefficients.add(new BezierCurveCoefficients(n, i));
        }
    }

    /**
     * This approximates the length of the Bezier curve in APPROXIMATION_STEPS number of steps
     *
     * @return returns the approximated length of the Bezier curve
     */
    public double approximateLength() {
        Point previousPoint = getPoint(0);
        Point currentPoint;
        double approxLength = 0;
        for (int i = 1; i <= APPROXIMATION_STEPS; i++) {
            currentPoint = getPoint(i/(double)APPROXIMATION_STEPS);
            approxLength += previousPoint.distanceFrom(currentPoint);
        }
        return approxLength;
    }

    /**
     * This returns the point on the Bezier curve that is specified by the parametric t value
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive
     * @return this returns the point requested
     */
    public Point getPoint(double t) {
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        double xCoordinate = 0;
        double yCoordinate = 0;

        // calculates the x coordinate of the point requested
        for (int i = 0; i < controlPoints.size(); i++) {
            xCoordinate += pointCoefficients.get(i).getValue(t) * controlPoints.get(i).getX();
        }

        // calculates the y coordinate of the point requested
        for (int i = 0; i < controlPoints.size(); i++) {
            yCoordinate += pointCoefficients.get(i).getValue(t) * controlPoints.get(i).getY();
        }
        return new Point(xCoordinate, yCoordinate, Point.CARTESIAN);
    }

    /**
     * This returns the curvature of the Bezier curve at a specified time point
     *
     * @param t the parametric time input
     * @return returns the curvature
     */
    public double getCurvature(double t) {
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        Vector derivative = new Vector(getDerivative(t).getR(), getDerivative(t).getTheta());
        Vector secondDerivative =  new Vector(getSecondDerivative(t).getR(), getSecondDerivative(t).getTheta());

        return (MathFunctions.crossProduct(derivative, secondDerivative))/Math.pow(derivative.getMagnitude(),3);
    }

    /**
     * This returns the derivative on the Bezier curve that is specified by the parametric t value
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive
     * @return this returns the derivative requested
     */
    public Point getDerivative(double t) {
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        double xCoordinate = 0;
        double yCoordinate = 0;

        // calculates the x coordinate of the point requested
        for (int i = 0; i < controlPoints.size()-1; i++) {
            xCoordinate += pointCoefficients.get(i).getDerivativeValue(t) * (MathFunctions.subtractPoints(controlPoints.get(i+1), controlPoints.get(i)).getX());
        }

        // calculates the y coordinate of the point requested
        for (int i = 0; i < controlPoints.size()-1; i++) {;
            yCoordinate += pointCoefficients.get(i).getDerivativeValue(t) * (MathFunctions.subtractPoints(controlPoints.get(i+1), controlPoints.get(i)).getY());
        }
        return new Point(xCoordinate, yCoordinate, Point.CARTESIAN);
    }

    /**
     * This returns the second derivative on the Bezier curve that is specified by the parametric t value
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive
     * @return this returns the second derivative requested
     */
    public Point getSecondDerivative(double t) {
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        double xCoordinate = 0;
        double yCoordinate = 0;

        // calculates the x coordinate of the point requested
        for (int i = 0; i < controlPoints.size()-2; i++) {
            xCoordinate += pointCoefficients.get(i).getSecondDerivativeValue(t) * (MathFunctions.addPoints(MathFunctions.subtractPoints(controlPoints.get(i+2), new Point(2*controlPoints.get(i+1).getX(), 2*controlPoints.get(i+1).getY(), Point.CARTESIAN)), controlPoints.get(i)).getX());
        }

        // calculates the y coordinate of the point requested
        for (int i = 0; i < controlPoints.size()-2; i++) {
            yCoordinate += pointCoefficients.get(i).getSecondDerivativeValue(t) * (MathFunctions.addPoints(MathFunctions.subtractPoints(controlPoints.get(i+2), new Point(2*controlPoints.get(i+1).getX(), 2*controlPoints.get(i+1).getY(), Point.CARTESIAN)), controlPoints.get(i)).getY());
        }
        return new Point(xCoordinate, yCoordinate, Point.CARTESIAN);
    }

    /**
     * Returns the ArrayList of control points for this Bezier curve
     *
     * @return This returns the ArrayList
     */
    public ArrayList<Point> getControlPoints() {
        return controlPoints;
    }

    /**
     * Returns the conversion factor of one unit of distance into time
     *
     * @return returns the conversion factor
     */
    public double UNIT_TO_TIME() {
        return UNIT_TO_TIME;
    }
}