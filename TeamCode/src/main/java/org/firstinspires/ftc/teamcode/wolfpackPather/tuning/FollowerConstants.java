package org.firstinspires.ftc.teamcode.wolfpackPather.tuning;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Vector;

@Config
public class FollowerConstants {

    // This section is for setting the actual drive vector for the front left wheel
    private static double xMovement = 1;
    private static double yMovement = 1;
    private static double[] convertToPolar = Point.cartesianToPolar(xMovement, yMovement);
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0],convertToPolar[1]));

    // Mass of robot in kilograms
    public static double mass = 0.0;

    // Heading PIDF coefficients
    public static double[] headingPIDFCoefficients = {0,0,0,0};

    // Translational PIDF coefficients
    public static double[] translationalPIDFCoefficients = {0,0,0,0};

    // Drive PIDF coefficients
    public static double[] drivePIDFCoefficients = {0,0,0,0};

    // Centrifugal force to power scaling
    public static double centrifugalScaling = 0;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    public static double zeroPowerAcceleration = -100;

    // When the drivetrain is at the end of its current path or path chain and the velocity goes
    // below this value, then end the path. This is in inches/second
    public static double pathEndVelocity = 0.01;

    // When the t-value of the closest point to the robot on the path is greater than this value,
    // then the path is considered at its end.
    public static double pathEndTValue = 0.99;
}
