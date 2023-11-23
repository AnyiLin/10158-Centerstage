package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

@Config
public class FollowerConstants {

    // This section is for setting the actual drive vector for the front left wheel
    private static double xMovement = 78.54602;
    private static double yMovement = -55.68387;
    private static double[] convertToPolar = Point.cartesianToPolar(xMovement, yMovement);
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0],convertToPolar[1]));

    // Mass of robot in kilograms
    public static double mass = 10.4326;

    // Large heading error PIDF coefficients
    public static CustomPIDFCoefficients largeHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            5,
            0,
            0,
            0);

    public static double headingPIDFSwitch = Math.PI/60;

    // Small heading error PIDF coefficients
    public static CustomPIDFCoefficients smallHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            15,
            3,
            0.3,
            0);

    // Small translational PIDF coefficients
    public static CustomPIDFCoefficients smallTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.9,
            0.5,
            0.1,
            0);

    public static double translationalPIDFSwitch = 1;

    // Small translational PIDF coefficients
    public static CustomPIDFCoefficients largeTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.6,
            0,
            0,
            0);

    // Drive PIDF coefficients
    public static CustomPIDFCoefficients drivePIDFCoefficients = new CustomPIDFCoefficients(
            0.75,
            0,
            0,
            0);

    // Centrifugal force to power scaling
    public static double centrifugalScaling = 2.5;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    public static double zeroPowerAcceleration = -29.5; // used to be -17.5

    // When the drivetrain is at the end of its current path or path chain and the velocity goes
    // below this value, then end the path. This is in inches/second
    // This can be custom set for each path
    public static double pathEndVelocity = 0.1;

    // When the drivetrain is at the end of its current path or path chain and the translational error goes
    // below this value, then end the path. This is in inches
    // This can be custom set for each path
    public static double pathEndTranslational = 0.5;

    // When the drivetrain is at the end of its current path or path chain and the heading error goes
    // below this value, then end the path. This is in radians
    // This can be custom set for each path
    public static double pathEndHeading = 0.01;

    // When the t-value of the closest point to the robot on the path is greater than this value,
    // then the path is considered at its end.
    public static double pathEndTValue = 0.99;
}
