package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

@Config
public class FollowerConstants {

    // This section is for setting the actual drive vector for the front left wheel
    private static double xMovement = 81.34056;
    private static double yMovement = -65.43028;
    private static double[] convertToPolar = Point.cartesianToPolar(xMovement, yMovement);
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0],convertToPolar[1]));

    // Mass of robot in kilograms
    public static double mass = 10.65942;

    // Large heading error PIDF coefficients
    public static CustomPIDFCoefficients largeHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            1,
            0,
            0,
            0);

    public static double largeHeadingPIDFFeedForward = 0.02;

    public static double headingPIDFSwitch = Math.PI/20;

    // Small heading error PIDF coefficients
    public static CustomPIDFCoefficients smallHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            5,
            0,
            0.1,
            0);

    public static double smallHeadingPIDFFeedForward = 0.02;

    // Small translational PIDF coefficients
    public static CustomPIDFCoefficients smallTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.4,
            0,
            0.01,
            0);

    // Small translational Integral value
    public static CustomPIDFCoefficients smallTranslationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);

    public static double smallTranslationalPIDFFeedForward = 0.035;

    public static double translationalPIDFSwitch = 3;

    // Large translational PIDF coefficients
    public static CustomPIDFCoefficients largeTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.1,
            0,
            0,
            0);

    public static double largeTranslationalPIDFFeedForward = 0.035;

    // Large translational Integral
    public static CustomPIDFCoefficients largeTranslationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);

    // Small drive PIDF coefficients
    public static CustomPIDFCoefficients smallDrivePIDFCoefficients = new CustomPIDFCoefficients(
            0.06,
            0,
            0.0001,
            0);

    public static double drivePIDFSwitch = 3;

    // Large drive PIDF coefficients
    public static CustomPIDFCoefficients largeDrivePIDFCoefficients = new CustomPIDFCoefficients(
            0.04,
            0,
            0.0001,
            0);

    // Centrifugal force to power scaling
    public static double centrifugalScaling = 0.0015;

    // Angular momentum (roughly) to heading scaling
    public static double angularMomentumScaling = 0;

    // Linear momentum to translational scaling
    public static double linearMomentumScaling = 0;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    // this is for straight lines
    public static double lineZeroPowerAcceleration = -50;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    // this is for curves
    public static double curvedZeroPowerAcceleration = -1;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    // this is for curves
    public static double forwardZeroPowerAcceleration = -34.62719;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    // this is for curves
    public static double lateralZeroPowerAcceleration = -78.15554;

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
    public static double pathEndHeading = 0.02;

    // When the t-value of the closest point to the robot on the path is greater than this value,
    // then the path is considered at its end.
    // This can be custom set for each path
    public static double pathEndTValue = 0.99;

    // When the path is considered at its end parametrically, then the follower has this many
    // seconds to further correct by default.
    // This can be custom set for each path
    public static double pathEndTimeout = 1.5;

    // When the drive power vector is less than this in magnitude, then
    public static double recalculateZeroPowerAccelerationLimit = 0.001;
}
