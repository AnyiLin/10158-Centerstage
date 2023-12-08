package org.firstinspires.ftc.teamcode.util;

import kotlin.jvm.JvmField;

public class CustomPIDFCoefficients {
    @JvmField public double P;
    @JvmField public double I;
    @JvmField public double D;
    @JvmField public double F;

    public FeedForwardConstant feedForwardConstantEquation;

    private boolean usingEquation;

    public CustomPIDFCoefficients(double p, double i, double d, double f) {
        P = p;
        I = i;
        D = d;
        F = f;
    }

    public CustomPIDFCoefficients(double p, double i, double d, FeedForwardConstant f) {
        usingEquation = true;
        P = p;
        I = i;
        D = d;
        feedForwardConstantEquation = f;
    }

    public double getCoefficient(double input) {
        if (!usingEquation) return F;
        return feedForwardConstantEquation.getConstant(input);
    }
}
