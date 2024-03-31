package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

public class Pose {
    private double x;
    private double y;
    private double heading;

    public Pose(double setX, double setY, double setHeading) {
        setX(setX);
        setY(setY);
        setHeading(setHeading);
    }

    public Pose() {
        this(0,0,0);
    }

    public void setX(double set) {
        x = set;
    }

    public void setY(double set) {
        y = set;
    }

    public void setHeading(double set) {
        heading = MathFunctions.normalizeAngle(set);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public Vector getVector() {
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(x, y);
        return returnVector;
    }

    public Vector getHeadingVector() {
        return new Vector(1, heading);
    }

    public void add(Pose pose) {
        setX(x + pose.getX());
        setY(y + pose.getY());
        setHeading(heading + pose.getHeading());
    }

    public void subtract(Pose pose) {
        setX(x - pose.getX());
        setY(y - pose.getY());
        setHeading(heading - pose.getHeading());
    }
}
