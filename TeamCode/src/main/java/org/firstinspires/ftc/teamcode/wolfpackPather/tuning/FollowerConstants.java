package org.firstinspires.ftc.teamcode.wolfpackPather.tuning;

import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Vector;

public class FollowerConstants {

    // This section is for setting the actual drive vector for the front left wheel
    private static double xMovement = 1;
    private static double yMovement = 1;
    private static double[] convertToPolar = Point.cartesianToPolar(xMovement, yMovement);
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0],convertToPolar[1]));
}
