package org.firstinspires.ftc.teamcode.wolfpackPather.follower;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.wolfpackPather.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.wolfpackPather.tuning.FollowerConstants;

import java.util.Arrays;
import java.util.List;

public class Follower {
    private HardwareMap hardwareMap;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private List<DcMotorEx> motors;

    private DriveVectorScaler driveVectorScaler;

    private PoseUpdater poseUpdater;

    private Pose2d closestPose;

    private Path currentPath;

    private PathChain currentPathChain;

    private final int BEZIER_CURVE_BINARY_STEP_LIMIT = 9;

    private int chainIndex;

    private boolean followingPathChain, isBusy;

    private double[] drivePowers;

    private PIDFController translationalXPIDF = new PIDFController(FollowerConstants.translationalPIDFCoefficients),
            translationalYPIDF = new PIDFController(FollowerConstants.translationalPIDFCoefficients),
            headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients),
            drivePIDF = new PIDFController(FollowerConstants.drivePIDFCoefficients);

    /**
     * This creates a new follower given a hardware map
     *
     * @param hardwareMap hardware map required
     */
    public Follower(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);
        poseUpdater = new PoseUpdater(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /**
     * This returns the current pose
     *
     * @return returns the pose
     */
    public Pose2d getPose() {
        return poseUpdater.getPose();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param pose the pose to set the starting pose to
     */
    public void setStartingPose(Pose2d pose) {
        poseUpdater.setStartingPose(pose);
    }


    /**
     * This follows a path
     *
     * @param path the path to follow
     */
    public void followPath(Path path) {
        isBusy = true;
        followingPathChain = false;
        currentPath = path;
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

    /**
     * This follows a path chain and only slows down at the end of the path chain
     *
     * @param pathChain the path chain to follow
     */
    public void followPathChain(PathChain pathChain) {
        isBusy = true;
        followingPathChain = true;
        chainIndex = 0;
        currentPathChain = pathChain;
        currentPath = pathChain.getPath(chainIndex);
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

    /**
     * Updates the robot's position and drive powers
     */
    public void update() {
        poseUpdater.update();
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
        if (currentPath.isAtEnd()) {
            if (followingPathChain && chainIndex < currentPathChain.size()-1) {
                // Not at last path, keep going
                chainIndex++;
                currentPath = currentPathChain.getPath(chainIndex);
            } else {
                // At last path, run some end detection stuff
                // set isBusy to false if at end
                if (poseUpdater.getVelocity().getMagnitude() < FollowerConstants.pathEndVelocity) {
                    isBusy = false;
                }
            }
        }

        if (isBusy) {
            drivePowers = driveVectorScaler.getDrivePowers(getCorrectiveVector(), getHeadingVector(), getDriveVector());

            for (int i = 0; i < motors.size(); i++) {
                motors.get(i).setPower(drivePowers[i]);
            }
        }
    }

    /**
     * This returns if the follower is currently following a path or a path chain
     *
     * @return returns if the follower is busy
     */
    public boolean isBusy() {
        return isBusy;
    }

    /**
     * This returns a Vector in the direction the robot must go to move along the path
     *
     * Note: This vector is clamped to be at most 1 in magnitude
     *
     * @return returns the drive vector
     */
    public Vector getDriveVector() {
        if (followingPathChain && chainIndex < currentPathChain.size()-1) {
            return new Vector(1, currentPath.getClosestPointHeadingGoal());
        }

        if (!currentPath.isAtEnd()) {
            drivePIDF.updateError(currentPath.length() * (1 - currentPath.getClosestPointTValue()) - getZeroPowerDistance());
        } else {
            Vector offset = new Vector(0,0);
            offset.setOrthogonalComponents(getPose().getX() - currentPath.getLastControlPoint().getX(), getPose().getY() - currentPath.getLastControlPoint().getY());
            drivePIDF.updateError(MathFunctions.dotProduct(currentPath.getEndTangent(), offset) - getZeroPowerDistance());
        }

        return new Vector(MathFunctions.clamp(drivePIDF.runPIDF(), -1, 1), currentPath.getClosestPointTangentVector().getTheta());
    }

    // TODO: remove
    public double zxcv() {
        return drivePIDF.getError();
    }

    /**
     * This returns the distance the robot is projected to go when all power is cut from the drivetrain
     * as a vector
     *
     * @return returns the projected distance vector
     */
    public double getZeroPowerDistance() {
        return -Math.pow(poseUpdater.getVelocity().getMagnitude(), 2) / (2 * FollowerConstants.zeroPowerAcceleration);
    }

    /**
     * This returns a Vector in the direction of the robot that contains the heading correction
     * as its magnitude
     *
     * Note: This vector is clamped to be at most 1 in magnitude
     *
     * @return returns the heading vector
     */
    public Vector getHeadingVector() {
        headingPIDF.updateError(MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) * MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()));
        return new Vector(MathFunctions.clamp(headingPIDF.runPIDF(), -1, 1), poseUpdater.getPose().getHeading());
    }

    /**
     * This returns a Vector in the direction the robot must go to account for both translational
     * error as well as centripetal force.
     *
     * Note: This vector is clamped to be at most 1 in magnitude
     *
     * @return returns the corrective vector
     */
    public Vector getCorrectiveVector() {
        Vector centripetal = getCentripetalForceCorrection();
        Vector translational = getTranslationalCorrection();
        Vector corrective = MathFunctions.addVectors(centripetal, translational);

        if (corrective.getMagnitude() > 1) {
            return MathFunctions.addVectors(centripetal, MathFunctions.scalarMultiplyVector(translational, driveVectorScaler.findNormalizingScaling(centripetal, translational)));
        }

        return corrective;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only translational
     * error
     *
     * Note: This vector is clamped to be at most 1 in magnitude
     *
     * @return returns the translational vector
     */
    public Vector getTranslationalCorrection() {
        Vector translationalVector = new Vector(0,0);
        translationalXPIDF.updateError(closestPose.getX() - poseUpdater.getPose().getX());
        translationalYPIDF.updateError(closestPose.getY() - poseUpdater.getPose().getY());
        translationalVector.setOrthogonalComponents(translationalXPIDF.runPIDF(), translationalYPIDF.runPIDF());
        translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), 0, 1));
        return translationalVector;
    }

    // TODO: remove later
    public double asdf() {
        return translationalYPIDF.getError();
    }

    public double qwerty() {
        return driveVectorScaler.getLeftSidePath().getMagnitude();
    }

    public double qwerty2() {
        return driveVectorScaler.getRightSidePath().getMagnitude();
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only centripetal
     * force
     *
     * Note: This vector is clamped to be between [0, 1] in magnitude
     *
     * @return returns the centripetal vector
     */
    public Vector getCentripetalForceCorrection() {
        double curvature = currentPath.getClosestPointCurvature();
        if (Double.isNaN(curvature)) return new Vector(0,0);
        return new Vector(MathFunctions.clamp(FollowerConstants.centrifugalScaling * FollowerConstants.mass * Math.pow(MathFunctions.dotProduct(poseUpdater.getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 2) * curvature,-1,1), currentPath.getClosestPointHeadingGoal() + MathFunctions.getSign(curvature) * (Math.PI/2));
    }

    public Pose2d getClosestPose() {
        return closestPose;
    }
}
