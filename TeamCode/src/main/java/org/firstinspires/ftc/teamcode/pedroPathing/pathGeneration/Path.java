package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

import java.util.ArrayList;

public class Path {
    private BezierCurve curve;

    private double startHeading, endHeading, closestPointCurvature, closestPointTValue, linearInterpolationEndTime;

    private Vector closestPointTangentVector, closestPointNormalVector;

    private boolean isTangentHeadingInterpolation = true, followTangentReversed;

    // When the drivetrain is at the end of its current path or path chain and the velocity goes
    // below this value, then end the path. This is in inches/second
    // This can be custom set for each path
    public static double pathEndVelocity = FollowerConstants.pathEndVelocity;

    // When the drivetrain is at the end of its current path or path chain and the translational error goes
    // below this value, then end the path. This is in inches
    // This can be custom set for each path
    public static double pathEndTranslational = FollowerConstants.pathEndTranslational;

    // When the drivetrain is at the end of its current path or path chain and the heading error goes
    // below this value, then end the path. This is in radians
    // This can be custom set for each path
    public static double pathEndHeading = FollowerConstants.pathEndHeading;

    // When the t-value of the closest point to the robot on the path is greater than this value,
    // then the path is considered at its end.
    // This can be custom set for each path
    public static double pathEndTValue = FollowerConstants.pathEndTValue;

    // When the path is considered at its end parametrically, then the follower has this many
    // seconds to further correct by default.
    // This can be custom set for each path
    public static double pathEndTimeout = FollowerConstants.pathEndTimeout;

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
        linearInterpolationEndTime = 1;
        isTangentHeadingInterpolation = false;
        this.startHeading = startHeading;
        this.endHeading = endHeading;
    }

    /**
     * This sets the heading interpolation to linear with a specified start heading for the path and
     * an end heading for the path
     *
     * @param startHeading the start heading for the path
     * @param endHeading the end heading for the path
     */
    public void setLinearHeadingInterpolation(double startHeading, double endHeading, double endTime) {
        linearInterpolationEndTime = MathFunctions.clamp(endTime, 0.000000001, 1);
        isTangentHeadingInterpolation = false;
        this.startHeading = startHeading;
        this.endHeading = endHeading;
    }

    /**
     * This sets the heading interpolation to maintain a constant heading
     *
     * @param setHeading the constant heading for the path
     */
    public void setConstantHeadingInterpolation(double setHeading) {
        linearInterpolationEndTime = 1;
        isTangentHeadingInterpolation = false;
        startHeading = setHeading;
        endHeading = setHeading;
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

        closestPointNormalVector = curve.getApproxSecondDerivative(closestPointTValue);

        closestPointCurvature = curve.getCurvature(closestPointTValue);

        return new Pose2d(returnPoint.getX(), returnPoint.getY(), getClosestPointHeadingGoal());
    }

    /**
     * This sets whether to follow the tangent heading reversed or not
     *
     * @param set sets reversed or not
     */
    public void setReversed(boolean set) {
        isTangentHeadingInterpolation = true;
        followTangentReversed = set;
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
     * This returns the normal vector at the closest point
     *
     * @return returns the normal vector at the closest point
     */
    public Vector getClosestPointNormalVector() {
        return MathFunctions.copyVector(closestPointNormalVector);
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
            if (followTangentReversed) return MathFunctions.normalizeAngle(closestPointTangentVector.getTheta() + Math.PI);
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
            if (followTangentReversed) return MathFunctions.normalizeAngle(curve.getDerivative(t).getTheta() + Math.PI);
            return curve.getDerivative(t).getTheta();
        } else {
            if (t > linearInterpolationEndTime) {
                return MathFunctions.normalizeAngle(endHeading);
            }
            return MathFunctions.normalizeAngle(startHeading + MathFunctions.getTurnDirection(startHeading, endHeading) * MathFunctions.getSmallestAngleDifference(endHeading, startHeading) * (t / linearInterpolationEndTime));
        }
    }

    /**
     * This returns if the robot is at the end of the path, according to the parametric t value
     *
     * @return returns if at end
     */
    public boolean isAtParametricEnd() {
        if (closestPointTValue >= pathEndTValue) return true;
        return false;
    }

    /**
     * This returns if the robot is at the beginning of the path, according to the parametric t value
     *
     * @return returns if at start
     */
    public boolean isAtParametricStart() {
        if (closestPointTValue <= 1-pathEndTValue) return true;
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

    /**
     * This sets the velocity stop criteria
     *
     * @param set
     */
    public void setPathEndVelocity(double set) {
        pathEndVelocity = set;
    }

    /**
     * This sets the translational stop criteria
     *
     * @param set
     */
    public void setPathEndTranslational(double set) {
        pathEndTranslational = set;
    }

    /**
     * This sets the heading stop criteria
     *
     * @param set
     */
    public void setPathEndHeading(double set) {
        pathEndHeading = set;
    }

    /**
     * This sets the parametric end criteria
     *
     * @param set
     */
    public void setPathEndTValue(double set) {
        pathEndTValue = set;
    }

    /**
     * This sets the path end timeout
     *
     * @param set
     */
    public void setPathEndTimeout(double set) {
        pathEndTimeout = set;
    }

    /**
     * This gets the velocity stop criteria
     */
    public double getPathEndVelocity() {
        return pathEndVelocity;
    }

    /**
     * This gets the translational stop criteria
     */
    public double getPathEndTranslational() {
        return pathEndTranslational;
    }

    /**
     * This gets the heading stop criteria
     */
    public double getPathEndHeading() {
        return pathEndHeading;
    }

    /**
     * This gets the parametric end criteria
     */
    public double getPathEndTValue() {
        return pathEndTValue;
    }

    /**
     * This gets the end correction time
     */
    public double getPathEndTimeout() {
        return pathEndTimeout;
    }

    /**
     * This returns the path type
     */
    public String pathType() {
        return curve.pathType();
    }
}