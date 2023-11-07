package org.firstinspires.ftc.teamcode.util;

import kotlin.jvm.JvmField;

public class CustomPIDFCoefficients {
    @JvmField public double P;
    @JvmField public double I;
    @JvmField public double D;
    @JvmField public double F;

    public CustomPIDFCoefficients(double p, double i, double d, double f) {
        P = p;
        I = i;
        D = d;
        F = f;
    }
}
