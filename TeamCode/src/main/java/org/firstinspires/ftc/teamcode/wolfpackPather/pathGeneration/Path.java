package org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Path {
    private BezierCurve curve;

    private double startHeading, endHeading, closestPointCurvature, closestPointTValue;

    private Vector closestPointTangentVector;

    private boolean isTangentHeadingInterpolation = true;

    public Path(BezierCurve curve) {
        this.curve = curve;
    }

    public void setLinearHeadingInterpolation(double startHeading, double endHeading) {
        this.startHeading = startHeading;
        this.endHeading = endHeading;
    }

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

    public Point getPoint(double t) {
        return curve.getPoint(t);
    }

    public double getClosestPointTValue() {
        return closestPointTValue;
    }

    public double length() {
        return curve.length();
    }

    public double getCurvature(double t) {
        return curve.getCurvature(t);
    }

    public double getClosestPointCurvature() {
        return closestPointCurvature;
    }

    public double getClosestPointHeadingGoal() {
        if (isTangentHeadingInterpolation) {
            return closestPointTangentVector.getTheta();
        } else {
            return getHeadingGoal(closestPointTValue);
        }
    }

    public double getHeadingGoal(double t) {
        if (isTangentHeadingInterpolation) {
            return curve.getDerivative(t).getTheta();
        } else {
            return MathFunctions.normalizeAngle(startHeading + getTurnDirection() * Math.abs(endHeading-startHeading) * t);
        }
    }

    public double getTurnDirection() {
        if (MathFunctions.normalizeAngle(endHeading-startHeading) >= 0 && MathFunctions.normalizeAngle(endHeading-startHeading) <= Math.PI * 2) {
            return 1; // counter clock wise
        }
        return -1; // clock wise
    }

    public boolean isAtEnd() {
        if (closestPointTValue >= 0.999) return true;
        return false;
    }
}