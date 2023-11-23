package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

public class BezierCurveCoefficients {
    private double coefficient, derivativeCoefficient, secondDerivativeCoefficient;
    private int n, i;

    /**
     * This creates the coefficients within the summation equations for calculating positions on
     * Bezier curves, the derivatives, and the second derivatives
     *
     * @param n this is the degree of the Bezier curve function
     * @param i this is the i within the summation equation
     */
    public BezierCurveCoefficients(int n, int i) {
        this.n = n;
        this.i = i;
        coefficient = MathFunctions.nCr(n, i);
        derivativeCoefficient = MathFunctions.nCr(n-1, i);
        secondDerivativeCoefficient = MathFunctions.nCr(n-2, i);
    }

    /**
     * This returns the coefficient for the summation to calculate a position on a Bezier curve
     *
     * @param t this is the time variable within the parametric equation for the Bezier curve
     * @return this returns the coefficient
     */
    public double getValue(double t) {
        return coefficient * Math.pow(1-t, n-i) * Math.pow(t, i);
    }

    /**
     * This returns the coefficient for the summation to calculate a derivative on a Bezier curve
     *
     * @param t this is the time variable within the parametric equation for the Bezier curve
     * @return this returns the coefficient
     */
    public double getDerivativeValue(double t) {
        return n * derivativeCoefficient * Math.pow(t, i) * Math.pow(1-t, n-i-1);
    }

    /**
     * This returns the coefficient for the summation to calculate a second derivative on a Bezier curve
     *
     * @param t this is the time variable within the parametric equation for the Bezier curve
     * @return this returns the coefficient
     */
    public double getSecondDerivativeValue(double t) {
        return n * (n-1) * secondDerivativeCoefficient * Math.pow(t, i) * Math.pow(1-t, n-i-2);
    }
}
