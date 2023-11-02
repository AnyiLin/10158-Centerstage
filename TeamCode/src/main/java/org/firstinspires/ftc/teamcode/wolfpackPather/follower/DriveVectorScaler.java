package org.firstinspires.ftc.teamcode.wolfpackPather.follower;

import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Vector;

public class DriveVectorScaler {
    // This is in the order left front, left back, right front, right back
    private Vector[] mecanumVectors;

    public DriveVectorScaler(Vector frontLeftVector) {
        mecanumVectors = new Vector[]{new Vector(frontLeftVector.getMagnitude(), frontLeftVector.getTheta()),
                new Vector(frontLeftVector.getMagnitude(), Math.PI-frontLeftVector.getTheta()),
                new Vector(frontLeftVector.getMagnitude(), Math.PI-frontLeftVector.getTheta()),
                new Vector(frontLeftVector.getMagnitude(), frontLeftVector.getTheta())};
    }

    /**
     * This takes in vectors for corrective power, heading power, and pathing power and outputs
     * an Array of four vectors, one for each wheel
     *
     * IMPORTANT NOTE: all vector inputs are clamped between 0 and 1 inclusive in magnitude
     *
     * @param correctivePower this vector includes the centrifugal force scaling vector as well as a
     * translational power vector to correct onto the Bezier curve the follower is following
     * @param headingPower the angle of this vector doesn't particularly matter, only the magnitude.
     * this tells the robot how much it should turn.
     * @param pathingPower this vector should always be of magnitude 1 and points in the direction that
     * we should be going along the path
     * @return
     */
    public Vector[] scaleDriveVector(Vector correctivePower, Vector headingPower, Vector pathingPower) {
        if (correctivePower.getMagnitude()>1) correctivePower.setMagnitude(1);
        if (correctivePower.getMagnitude()>1) correctivePower.setMagnitude(1);

        // this is an offset from the preferred drive direction (forward)
        double actualPathingTheta = Math.asin(correctivePower.getMagnitude()/ pathingPower.getMagnitude());
        //TODO: first correct the pathing power using the method shown in the wolfpack video at
        // about thirteen minutes. Then, calculate wheel powers. Afterwards, add on the heading
        // values to each wheel and normalize all wheel powers to be of 1 magnitude maximum
        return null;
    }
}
