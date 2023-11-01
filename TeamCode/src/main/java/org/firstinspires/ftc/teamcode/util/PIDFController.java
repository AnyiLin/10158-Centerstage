package org.firstinspires.ftc.teamcode.util;

public class PIDFController {
    private double[] coefficients = new double[4];

    private double previousError, error, position, targetPosition, accumulatedError;

    public PIDFController(double P, double I, double D, double F) {
        coefficients[0] = P;
        coefficients[1] = I;
        coefficients[2] = D;
        coefficients[3] = F;
        previousError = 0;
        error = 0;
        position = 0;
        targetPosition = 0;
        accumulatedError = 0;
    }

    public double runPIDF() {
        return error*P() + (error-previousError)*D() + accumulatedError*I() + F();
    }

    public void updatePosition(double update) {
        position = update;
        previousError = error;
        error = targetPosition-position;
        accumulatedError += error;
    }

    public void setTargetPosition(double set) {
        targetPosition = set;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setCoefficients(double[] set) {
        coefficients = set;
    }

    public double[] getCoefficients() {
        return coefficients;
    }

    public void setP(double set) {
        coefficients[0] = set;
    }

    public double P() {
        return coefficients[0];
    }

    public void setI(double set) {
        coefficients[1] = set;
    }

    public double I() {
        return coefficients[1];
    }

    public void setD(double set) {
        coefficients[2] = set;
    }

    public double D() {
        return coefficients[2];
    }

    public void setF(double set) {
        coefficients[3] = set;
    }

    public double F() {
        return coefficients[3];
    }
}
