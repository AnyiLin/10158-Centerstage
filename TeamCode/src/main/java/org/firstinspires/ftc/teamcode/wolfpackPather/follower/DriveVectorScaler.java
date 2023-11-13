package org.firstinspires.ftc.teamcode.wolfpackPather.follower;

import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Vector;

public class DriveVectorScaler {
    // This is in the order left front, left back, right front, right back. These are also normalized
    private Vector[] mecanumVectors;

    /**
     * This creates a new DriveVectorScaler, which takes in various movement vectors and outputs
     * the wheel drive powers necessary to move in the intended direction, given a set of preferred
     * wheel drive vectors
     *
     * @param frontLeftVector this is the front left mecanum wheel's preferred drive vector
     */
    public DriveVectorScaler(Vector frontLeftVector) {
        Vector copiedFrontLeftVector = MathFunctions.normalizeVector(frontLeftVector);
        mecanumVectors = new Vector[]{
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2*Math.PI-copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2*Math.PI-copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())};
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
    public double[] getDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() > 1) correctivePower.setMagnitude(1);
        if (headingPower.getMagnitude() > 1) headingPower.setMagnitude(1);
        if (pathingPower.getMagnitude() > 1) pathingPower.setMagnitude(1);

        // This contains a copy of the mecanum wheel vectors
        Vector[] mecanumVectorsCopy = new Vector[4];

        // this contains the pathing vectors, one for each side (heading control requires 2)
        Vector[] truePathingVectors = new Vector[2];

        if (correctivePower.getMagnitude() == 1) {
            // checks for corrective power equal to 1 in magnitude. if equal to one, then set pathing power to that
            truePathingVectors[0] = MathFunctions.copyVector(correctivePower);
            truePathingVectors[1] = MathFunctions.copyVector(correctivePower);
        } else {
            // corrective power did not take up all the power, so add on heading power
            Vector leftSideVector = MathFunctions.subtractVectors(correctivePower, headingPower);
            Vector rightSideVector = MathFunctions.addVectors(correctivePower, headingPower);

            if (leftSideVector.getMagnitude() > 1 || rightSideVector.getMagnitude() > 1) {
                //if the combined corrective and heading power is greater than 1, then scale down heading power
                double headingScalingFactor = Math.min(findNormalizingScaling(correctivePower, headingPower), findNormalizingScaling(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, -1)));
                truePathingVectors[0] = MathFunctions.subtractVectors(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, headingScalingFactor));
                truePathingVectors[1] = MathFunctions.addVectors(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, headingScalingFactor));
            } else {
                // if we're here then we can add on some drive power but scaled down to 1
                Vector leftSideVectorWithPathing = MathFunctions.addVectors(leftSideVector, pathingPower);
                Vector rightSideVectorWithPathing = MathFunctions.addVectors(rightSideVector, pathingPower);

                if (leftSideVectorWithPathing.getMagnitude() > 1 || rightSideVectorWithPathing.getMagnitude() > 1) {
                    // too much power now, so we scale down the pathing vector
                    double pathingScalingFactor = Math.min(findNormalizingScaling(leftSideVector, pathingPower), findNormalizingScaling(rightSideVector, pathingPower));
                    truePathingVectors[0] = MathFunctions.addVectors(leftSideVector, MathFunctions.scalarMultiplyVector(pathingPower, pathingScalingFactor));
                    truePathingVectors[1] = MathFunctions.addVectors(rightSideVector, MathFunctions.scalarMultiplyVector(pathingPower, pathingScalingFactor));
                } else {
                    // just add the vectors together and you get the final vector
                    truePathingVectors[0] = MathFunctions.copyVector(leftSideVectorWithPathing);
                    truePathingVectors[1] = MathFunctions.copyVector(rightSideVectorWithPathing);
                }
            }
        }

        // TODO: remove
        leftSidePath = MathFunctions.copyVector(truePathingVectors[0]);

        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            // this copies the vectors from mecanumVectors but creates new references for them
            mecanumVectorsCopy[i] = MathFunctions.copyVector(mecanumVectors[i]);

            // this rotates the vectors by the angle of the actual pathing vector so we can get their vertical components
            if (i == 0 || i == 1) {
                mecanumVectorsCopy[i].rotateVector(-truePathingVectors[0].getTheta());
            } else {
                mecanumVectorsCopy[i].rotateVector(-truePathingVectors[1].getTheta());
            }
        }

        // These two statements scale the wheel vectors so that the vertical components cancel out
        // it also scales the wheel vectors to the magnitude of the pathing vectors
        if (Math.abs(mecanumVectorsCopy[0].getYComponent()) < Math.abs(mecanumVectorsCopy[1].getYComponent())) {
            mecanumVectorsCopy[1].setMagnitude(truePathingVectors[0].getMagnitude() * (-mecanumVectorsCopy[0].getYComponent() / mecanumVectorsCopy[1].getYComponent()));
            mecanumVectorsCopy[0].setMagnitude(truePathingVectors[0].getMagnitude());
        } else {
            mecanumVectorsCopy[0].setMagnitude(truePathingVectors[0].getMagnitude() * (-mecanumVectorsCopy[1].getYComponent() / mecanumVectorsCopy[0].getYComponent()));
            mecanumVectorsCopy[1].setMagnitude(truePathingVectors[0].getMagnitude());
        }
        if (Math.abs(mecanumVectorsCopy[2].getYComponent()) < Math.abs(mecanumVectorsCopy[3].getYComponent())) {
            mecanumVectorsCopy[3].setMagnitude(truePathingVectors[1].getMagnitude() * (-mecanumVectorsCopy[2].getYComponent() / mecanumVectorsCopy[3].getYComponent()));
            mecanumVectorsCopy[2].setMagnitude(truePathingVectors[1].getMagnitude());
        } else {
            mecanumVectorsCopy[2].setMagnitude(truePathingVectors[1].getMagnitude() * (-mecanumVectorsCopy[3].getYComponent() / mecanumVectorsCopy[2].getYComponent()));
            mecanumVectorsCopy[3].setMagnitude(truePathingVectors[1].getMagnitude());
        }

        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            // this rotates the vectors back to make the math easier
            if (i == 0 || i == 1) {
                mecanumVectorsCopy[i].rotateVector(truePathingVectors[0].getTheta());
            } else {
                mecanumVectorsCopy[i].rotateVector(truePathingVectors[1].getTheta());
            }
        }

        return new double[] {MathFunctions.dotProduct(mecanumVectorsCopy[0], mecanumVectors[0]), MathFunctions.dotProduct(mecanumVectorsCopy[1], mecanumVectors[1]), MathFunctions.dotProduct(mecanumVectorsCopy[2], mecanumVectors[2]), MathFunctions.dotProduct(mecanumVectorsCopy[3], mecanumVectors[3])};
    }

    // TODO: remove
    private Vector leftSidePath;

    public Vector getLeftSidePath() {
        return leftSidePath;
    }

    /**
     * This takes in two vectors, one static and one variable, and returns the scaling factor that,
     * when multiplied to the variable vector, results in magnitude of the sum of the static vector
     * and the scaled variable vector being 1
     *
     * IMPORTANT NOTE: I did not intend for this to be used for anything other than the method above
     * this one in this class, so there will be errors if you input vectors of length greater than 1,
     * and it will scale up the variable vector if the magnitude of the sum of the two input vectors
     * isn't greater than 1. So, just don't use this elsewhere. There's gotta be a better way to do
     * whatever you're trying to do.
     *
     * @param staticVector the vector that should not be changed
     * @param variableVector the vector getting scaled
     * @return returns the scaling factor
     */
    public double findNormalizingScaling(Vector staticVector, Vector variableVector) {
            double a = Math.pow(variableVector.getXComponent(), 2) + Math.pow(variableVector.getYComponent(), 2);
            double b = staticVector.getXComponent() * variableVector.getXComponent() + staticVector.getYComponent() * variableVector.getYComponent();
            double c = Math.pow(staticVector.getXComponent(), 2) + Math.pow(staticVector.getYComponent(), 2) - 1.0;
            return (-b +  Math.sqrt(Math.pow(b, 2) - a*c))/(a);
    }
}
