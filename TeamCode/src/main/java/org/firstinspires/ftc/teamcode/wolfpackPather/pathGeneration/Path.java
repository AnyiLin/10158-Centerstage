package org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.wolfpackPather.tuning.FollowerConstants;

import java.util.ArrayList;

public class Path {
    private BezierCurve curve;

    private double startHeading, endHeading, closestPointCurvature, closestPointTValue;

    private Vector closestPointTangentVector;

    private boolean isTangentHeadingInterpolation = true;

    /**
     * Creates a new Path from a Bezier curve. The default heading interpolation is tangential.
     *
     * @param curve the Bezier curve
     */
    public Path(BezierCurve curve) {
        this.curve = curve;
    }

    /**
     * This sets the heading interpolation to linear with a specified start heading for the path and
     * an end heading for the path
     *
     * @param startHeading the start heading for the path
     * @param endHeading the end heading for the path
     */
    public void setLinearHeadingInterpolation(double startHeading, double endHeading) {
        this.startHeading = startHeading;
        this.endHeading = endHeading;
    }

    /**
     * This gets the closest point from a specified pose to the curve with a specified binary search
     * step limit
     *
     * @param pose the pose
     * @param searchStepLimit the binary search step limit
     * @return returns the closest point
     */
    public Pose2d getClosestPoint(Pose2d pose, int searchStepLimit) {
        double lower = 0;
        double upper = 1;
        Point returnPoint;

        // we don't need to calculate the midpoint, so we start off at the 1/4 and 3/4 point
        for (int i = 0; i < searchStepLimit; i++) {
            if (MathFunctions.distance(pose, getPoint(lower + 0.25 * (upper-lower))) > MathFunctions.distance(pose, getPoint(lower + 0.75 * (upper-lower)))) {
                lower += (upper-lower)/2.0;
            } else {
                upper -= (upper-lower)/2.0;
            }
        }

        closestPointTValue = lower + 0.5 * (upper-lower);

        returnPoint = getPoint(closestPointTValue);

        closestPointTangentVector = curve.getDerivative(closestPointTValue);

        closestPointCurvature = curve.getCurvature(closestPointTValue);

        return new Pose2d(returnPoint.getX(), returnPoint.getY(), getClosestPointHeadingGoal());
    }

    /**
     * This returns the unit tangent vector at the end of the curve
     *
     * @return returns the tangent vector
     */
    public Vector getEndTangent() {
        return curve.getEndTangent();
    }

    /**
     * This gets a point on the curve at a specified t-value
     *
     * @param t the t-value specified
     * @return returns the point at the t-value point on the curve
     */
    public Point getPoint(double t) {
        return curve.getPoint(t);
    }

    /**
     * This returns the t-value of the closest point on the curve
     *
     * @return returns the closest point t-value
     */
    public double getClosestPointTValue() {
        return closestPointTValue;
    }

    /**
     * This returns the approximated length of the curve
     *
     * @return returns the length of the curve
     */
    public double length() {
        return curve.length();
    }

    /**
     * This returns the curvature of the curve at a specified t-value
     *
     * @param t the specified t-value
     * @return returns the curvature of the curve at the specified t-value
     */
    public double getCurvature(double t) {
        return curve.getCurvature(t);
    }

    /**
     * This returns the curvature of the curve at the closest point
     *
     * @return returns the curvature of the curve at the closest point
     */
    public double getClosestPointCurvature() {
        return closestPointCurvature;
    }

    /**
     * This returns the tangent vector at the closest point
     *
     * @return returns the tangent vector at the closest point
     */
    public Vector getClosestPointTangentVector() {
        return MathFunctions.copyVector(closestPointTangentVector);
    }

    /**
     * This returns the heading goal at the closest point
     *
     * @return returns the heading goal at the closest point
     */
    public double getClosestPointHeadingGoal() {
        if (isTangentHeadingInterpolation) {
            return closestPointTangentVector.getTheta();
        } else {
            return getHeadingGoal(closestPointTValue);
        }
    }

    /**
     * This gets the heading goal at a specified t-value
     *
     * @param t the specified t-value
     * @return returns the heading goal at the specified t-value
     */
    public double getHeadingGoal(double t) {
        if (isTangentHeadingInterpolation) {
            return curve.getDerivative(t).getTheta();
        } else {
            return MathFunctions.normalizeAngle(startHeading + MathFunctions.getTurnDirection(startHeading, endHeading) * Math.abs(endHeading-startHeading) * t);
        }
    }

    /**
     * This returns if the robot is at the end of the path
     *
     * @return returns if at end
     */
    public boolean isAtEnd() {
        if (closestPointTValue >= FollowerConstants.pathEndTValue) return true;
        return false;
    }

    /**
     * Returns the ArrayList of control points for this Bezier curve
     *
     * @return This returns the ArrayList
     */
    public ArrayList<Point> getControlPoints() {
        return curve.getControlPoints();
    }

    /**
     * Returns the first control point for this Bezier curve
     *
     * @return This returns the Point
     */
    public Point getFirstControlPoint() {
        return curve.getFirstControlPoint();
    }

    /**
     * Returns the second control point, or the one after the start, for this Bezier curve
     *
     * @return This returns the Point
     */
    public Point getSecondControlPoint() {
        return curve.getSecondControlPoint();
    }

    /**
     * Returns the second to last control point for this Bezier curve
     *
     * @return This returns the Point
     */
    public Point getSecondToLastControlPoint() {
        return curve.getSecondToLastControlPoint();
    }

    /**
     * Returns the last control point for this Bezier curve
     *
     * @return This returns the Point
     */
    public Point getLastControlPoint() {
        return curve.getLastControlPoint();
    }
}