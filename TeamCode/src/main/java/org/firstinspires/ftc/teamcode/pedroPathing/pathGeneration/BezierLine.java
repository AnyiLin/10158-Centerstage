package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import java.util.ArrayList;

public class BezierLine extends BezierCurve {

    private Point startPoint, endPoint;

    private Vector endTangent;

    private double UNIT_TO_TIME, length;

    /**
     * This creates a new BezierLine with specified start and end points
     * This is just a line but it extends the BezierCurve class so things work
     *
     * @param startPoint start point of the line
     * @param endPoint end point of the line
     */
    public BezierLine(Point startPoint, Point endPoint) {
        super();
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        length = approximateLength();
        UNIT_TO_TIME = 1/length;
        endTangent = MathFunctions.normalizeVector(getDerivative(1));
    }

    /**
     * This returns the unit tangent vector at the end of the line
     *
     * @return returns the tangent vector
     */
    @Override
    public Vector getEndTangent() {
        return MathFunctions.copyVector(endTangent);
    }

    /**
     * This gets the length of the Bezier line
     *
     * @return returns the length of the Bezier line
     */
    @Override
    public double approximateLength() {
        return Math.sqrt(Math.pow(startPoint.getX()-endPoint.getX(), 2) + Math.pow(startPoint.getY()-endPoint.getY(), 2));
    }

    /**
     * This returns the point on the Bezier line that is specified by the parametric t value
     *
     * @param t this is the t value of the parametric line. t is clamped to be between 0 and 1 inclusive
     * @return this returns the point requested
     */
    @Override
    public Point getPoint(double t) {
        t = MathFunctions.clamp(t, 0, 1);
        return new Point((endPoint.getX()-startPoint.getX())*t+startPoint.getX(), (endPoint.getY()-startPoint.getY())*t+startPoint.getY(), Point.CARTESIAN);
    }

    /**
     * This returns the curvature of the Bezier line (0.0)
     *
     * @param t the parametric time input
     * @return returns the curvature (0.0)
     */
    @Override
    public double getCurvature(double t) {
        return 0.0;
    }

    /**
     * This returns the derivative on the Bezier line (it's constant)
     * The t value doesn't really do anything, but it's there so I can override methods
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive
     * @return this returns the derivative requested
     */
    @Override
    public Vector getDerivative(double t) {
        Vector returnVector = new Vector();

        returnVector.setOrthogonalComponents(endPoint.getX()-startPoint.getX(), endPoint.getY()-startPoint.getY());

        return returnVector;
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
        returnList.add(startPoint);
        returnList.add(endPoint);
        return returnList;
    }

    /**
     * Returns the first control point for this Bezier line
     *
     * @return This returns the Point
     */
    @Override
    public Point getFirstControlPoint() {
        return startPoint;
    }

    /**
     * Returns the second control point, or the one after the start, for this Bezier line
     *
     * @return This returns the Point
     */
    @Override
    public Point getSecondControlPoint() {
        return endPoint;
    }

    /**
     * Returns the second to last control point for this Bezier line
     *
     * @return This returns the Point
     */
    @Override
    public Point getSecondToLastControlPoint() {
        return startPoint;
    }

    /**
     * Returns the last control point for this Bezier line
     *
     * @return This returns the Point
     */
    @Override
    public Point getLastControlPoint() {
        return endPoint;
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
        return UNIT_TO_TIME;
    }

    /**
     * Returns the type of path
     *
     * @return returns the type of path
     */
    @Override
    public String pathType() {
        return "line";
    }
}
