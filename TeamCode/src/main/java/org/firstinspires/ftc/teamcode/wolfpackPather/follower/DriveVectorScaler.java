package org.firstinspires.ftc.teamcode.wolfpackPather.follower;

import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Vector;

public class DriveVectorScaler {
    // This is in the order left front, left back, right front, right back. These are also normalized
    private Vector[] mecanumVectors;

    public DriveVectorScaler(Vector frontLeftVector) {
        frontLeftVector = MathFunctions.normalizeVector(frontLeftVector);
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
        //TODO: first correct the pathing power using the method shown in the wolfpack video at
        // about thirteen minutes. Then, calculate wheel powers. Afterwards, add on the heading
        // values to each wheel and normalize all wheel powers to be of 1 magnitude maximum

        // this is the corrected pathing power, as shown in wolfpack's video at about 13 minutes
        Vector actualPathingVector = new Vector(Math.sqrt(correctivePower.getMagnitude()*correctivePower.getMagnitude()+pathingPower.getMagnitude()*pathingPower.getMagnitude()), pathingPower.getTheta()+Math.asin(correctivePower.getMagnitude()/ pathingPower.getMagnitude()));

        Vector[] mecanumVectors = new Vector[4];
        double smallestVerticalComponent = 1;

        for (int i = 0; i < mecanumVectors.length; i++) {
            // this copies the vectors from mecanumVectors but creates new references for them
            mecanumVectors[i] = MathFunctions.scalarMultiplyVector(this.mecanumVectors[i], 1);

            // this rotates the vectors by the angle of the actual pathing vector so we can get their vertical components
            mecanumVectors[i].rotateVector(-actualPathingVector.getTheta());

            //gets the smallest vertical component
            if (Math.abs(mecanumVectors[i].getYComponent()) < smallestVerticalComponent) smallestVerticalComponent = Math.abs(mecanumVectors[i].getYComponent());
        }

        /* this scales down all the vertical components of the mecanum wheel vectors so that the
            robot drives in the direction of the actual pathing vector
         */
        for (int i = 0; i < mecanumVectors.length; i++) {
            mecanumVectors[i] = MathFunctions.scalarMultiplyVector(mecanumVectors[i], smallestVerticalComponent/Math.abs(mecanumVectors[i].getYComponent()));
        }


        return null;
    }
}
