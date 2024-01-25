package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import java.util.ArrayList;

public class BezierPoint extends BezierCurve {

    private Point point;

    private Vector endTangent = new Vector();

    private double UNIT_TO_TIME, length;

    /**
     * This creates a new BezierPoint with specified start and end points
     * This is just a point but it extends the BezierCurve class so things work
     *
     * @param point the one point
     */
    public BezierPoint(Point point) {
        super();
        this.point = point;
        length = approximateLength();
    }

    /**
     * This returns the unit tangent vector at the end of the point??
     * Somehow?
     *
     * @return returns the tangent vector
     */
    @Override
    public Vector getEndTangent() {
        return MathFunctions.copyVector(endTangent);
    }

    /**
     * This sets the end tangent Vector if you want to change the movement behavior of the robot
     */
    public void setEndTangent(Vector set) {
        endTangent = MathFunctions.copyVector(set);
    }

    /**
     * This gets the length of the Bezier line
     *
     * @return returns the length of the Bezier line
     */
    @Override
    public double approximateLength() {
        return 0.0;
    }

    /**
     * This returns the point on the Bezier line that is specified by the parametric t value
     *
     * @param t this is the t value of the parametric line. t is clamped to be between 0 and 1 inclusive
     * @return this returns the point requested
     */
    @Override
    public Point getPoint(double t) {
        return new Point(point.getX(), point.getY(), Point.CARTESIAN);
    }

    /**
     * This returns the curvature of the Bezier point (0.0)
     *
     * @param t the parametric time input
     * @return returns the curvature (0.0)
     */
    @Override
    public double getCurvature(double t) {
        return 0.0;
    }

    /**
     * This returns the derivative on the Bezier line (0 vector)
     * The t value doesn't really do anything, but it's there so I can override methods
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive
     * @return this returns the derivative requested (0 vector)
     */
    @Override
    public Vector getDerivative(double t) {
        return MathFunctions.copyVector(endTangent);
    }

    /**
     * This returns the second derivative on the Bezier line (0 vector)
     * Once again, the t is only there for the override
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive
     * @return this returns the second derivative requested (0 vector)
     */
    @Override
    public Vector getSecondDerivative(double t) {
        return new Vector();
    }

    @Override
    public Vector getApproxSecondDerivative(double t) {
        return new Vector();
    }

    /**
     * Returns the ArrayList of control points for this Bezier line
     *
     * @return This returns the ArrayList
     */
    @Override
    public ArrayList<Point> getControlPoints() {
        ArrayList<Point> returnList = new ArrayList<>();
        returnList.add(point);
        return returnList;
    }

    /**
     * Returns the first control point for this Bezier point
     *
     * @return This returns the Point
     */
    @Override
    public Point getFirstControlPoint() {
        return point;
    }

    /**
     * Returns the second control point, or the one after the start, for this Bezier point
     *
     * @return This returns the Point
     */
    @Override
    public Point getSecondControlPoint() {
        return point;
    }

    /**
     * Returns the second to last control point for this Bezier point
     *
     * @return This returns the Point
     */
    @Override
    public Point getSecondToLastControlPoint() {
        return point;
    }

    /**
     * Returns the last control point for this Bezier point
     *
     * @return This returns the Point
     */
    @Override
    public Point getLastControlPoint() {
        return point;
    }

    /**
     * Returns the length of this Bezier line
     *
     * @return This returns the length
     */
    @Override
    public double length() {
        return length;
    }

    /**
     * Returns the conversion factor of one unit of distance into time
     *
     * @return returns the conversion factor
     */
    @Override
    public double UNIT_TO_TIME() {
        return 0;
    }

    /**
     * Returns the type of path
     *
     * @return returns the type of path
     */
    @Override
    public String pathType() {
        return "point";
    }
}
