package org.firstinspires.ftc.teamcode.wolfpackPather.follower;

import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Vector;

public class DriveVectorScaler {
    // This is in the order left front, left back, right front, right back. These are also normalized
    private Vector[] mecanumVectors;

    // This tells the scaler how much
    private final double MAXIMUM_TRANSLATIONAL_POWER_PORTION = 0.65;

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
        if (correctivePower.getMagnitude() > 1) correctivePower.setMagnitude(1);
        if (headingPower.getMagnitude() > 1) headingPower.setMagnitude(1);
        if (pathingPower.getMagnitude() > 1) pathingPower.setMagnitude(1);

        // checks for corrective power equal to 1 in magnitude
        if (correctivePower.getMagnitude() == 1) {

        }




        // this is the corrected pathing theta, as shown in wolfpack's video at about 13 minutes
        double actualPathingMagnitude = Math.sqrt(correctivePower.getMagnitude()*correctivePower.getMagnitude()+pathingPower.getMagnitude()*pathingPower.getMagnitude());
        if (actualPathingMagnitude > 1) {
            actualPathingMagnitude = 1;
            pathingPower.setMagnitude(Math.sqrt(1-correctivePower.getMagnitude()*correctivePower.getMagnitude()));
        }
        double actualPathingTheta = 0;
        if (actualPathingMagnitude != 0) {
            if (pathingPower.getXComponent() < 0) {
                actualPathingTheta = Math.PI-Math.asin(correctivePower.getMagnitude() / actualPathingMagnitude);
            } else if (pathingPower.getYComponent() < 0) {
                actualPathingTheta = 2*Math.PI+Math.asin(correctivePower.getMagnitude() / actualPathingMagnitude);
            } else {
                actualPathingTheta = Math.asin(correctivePower.getMagnitude() / actualPathingMagnitude);
            }
        }


        Vector[] mecanumVectors = new Vector[4];
        Vector[] truePathingVectors = new Vector[2];
        double smallestVerticalComponent = 1;

        for (int i = 0; i < mecanumVectors.length; i++) {
            // this copies the vectors from mecanumVectors but creates new references for them
            mecanumVectors[i] = MathFunctions.scalarMultiplyVector(this.mecanumVectors[i], 1);

            // this rotates the vectors by the angle of the actual pathing vector so we can get their vertical components
            mecanumVectors[i].rotateVector(-actualPathingTheta);

            //gets the smallest vertical component
            if (Math.abs(mecanumVectors[i].getYComponent()) < smallestVerticalComponent) smallestVerticalComponent = Math.abs(mecanumVectors[i].getYComponent());
        }

        for (int i = 0; i < mecanumVectors.length; i++) {
        /* this scales down all the vertical components of the mecanum wheel vectors so that the
            robot drives in the direction of the actual pathing vector
         */
            mecanumVectors[i] = MathFunctions.scalarMultiplyVector(mecanumVectors[i], (smallestVerticalComponent/Math.abs(mecanumVectors[i].getYComponent())) * 1);

            // this adds on the heading powers to the wheels and scales the largest vector up to 1
            // note: a positive heading power value means that we need to turn left
            if (i == 0 || i == 1) {
                mecanumVectors[i].setMagnitude(mecanumVectors[i].getMagnitude() - headingPower.getMagnitude());
            } else {
                mecanumVectors[i].setMagnitude(mecanumVectors[i].getMagnitude() + headingPower.getMagnitude());
            }

            mecanumVectors[i] = MathFunctions.normalizeVector(mecanumVectors[i]);
        }

        return mecanumVectors;
    }
}
