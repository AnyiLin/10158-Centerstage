package org.firstinspires.ftc.teamcode.lupinePather.pathGeneration;

import java.util.ArrayList;

public class BezierCurve {
    private ArrayList<ArrayList<Point>> curve = new ArrayList<ArrayList<Point>>();

    private ArrayList<Point> controlPoints = new ArrayList<Point>();

    public BezierCurve(ArrayList<Point> controlPoints) {
        this.controlPoints = controlPoints;
    }

    public ArrayList<Point> getControlPoints() {
        return controlPoints;
    }
}