package org.firstinspires.ftc.teamcode.wolfpackPather.tuning;

import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Vector;

public class FollowerConstants {

    // This section is for setting the actual drive vector for the front left wheel
    private static double xMovement = 1;
    private static double yMovement = 1;
    private static double[] convertToPolar = Point.cartesianToPolar(xMovement, yMovement);
    public static final Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0],convertToPolar[1]));

    // Mass of robot in kilograms
    public static final double mass = 0.0;

    // Heading PIDF coefficients
    public static double[] headingPIDFCoefficients = {0,0,0,0};

    // Translational PIDF coefficients
    public static double[] translationalPIDFCoefficients = {0,0,0,0};

    // Drive PIDF coefficients
    public static double[] drivePIDFCoefficients = {0,0,0,0};

    // Centrifugal force to power scaling
    public static double centrifugalScaling = 1;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    public static double zeroPowerAcceleration = -100;
}
