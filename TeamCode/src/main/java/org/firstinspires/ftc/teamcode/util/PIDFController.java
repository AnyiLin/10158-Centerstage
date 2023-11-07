package org.firstinspires.ftc.teamcode.util;

public class PIDFController {
    private CustomPIDFCoefficients coefficients;

    private double previousError, error, position, targetPosition, accumulatedError;

    public PIDFController(CustomPIDFCoefficients set) {
        coefficients = set;
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

    public void updateError(double error) {
        previousError = this.error;
        this.error = error;
        accumulatedError += error;
    }

    public void reset() {
        previousError = 0;
        error = 0;
        position = 0;
        targetPosition = 0;
        accumulatedError = 0;
    }

    public void setTargetPosition(double set) {
        targetPosition = set;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setCoefficients(CustomPIDFCoefficients set) {
        coefficients = set;
    }

    public CustomPIDFCoefficients getCoefficients() {
        return coefficients;
    }

    public void setP(double set) {
        coefficients.P = set;
    }

    public double P() {
        return coefficients.P;
    }

    public void setI(double set) {
        coefficients.I = set;
    }

    public double I() {
        return coefficients.I;
    }

    public void setD(double set) {
        coefficients.D = set;
    }

    public double D() {
        return coefficients.D;
    }

    public void setF(double set) {
        coefficients.F = set;
    }

    public double F() {
        return coefficients.F;
    }
}
