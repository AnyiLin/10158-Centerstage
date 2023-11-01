package org.firstinspires.ftc.teamcode.lupinePather.pathGeneration;

public class MathFunctions {

    /**
     * There is possibly a more efficient method of doing this, but I'm doing it this way
     * because it's simple and doesn't have many rounding errors in the middle
     *
     * @param n this is how many you want to choose from
     * @param r this is how many you want to choose
     * @return returns the result of the "n choose r" function
     */
    public static double nCr(int n, int r) {
        // this multiplies up the numerator of the nCr function
        double num = 1;
        for (int counter = n; counter > n-r; counter--) {
            num *= n;
        }

        // this multiplies up the denominator of the nCr function
        double denom = 1;
        for (int counter = 1; counter <=r; counter++) {
            denom *= counter;
        }

        return num/denom;
    }
}
