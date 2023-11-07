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

    private PIDFController translationalPIDF = new PIDFController(FollowerConstants.translationalPIDFCoefficients),
            headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients),
            drivePIDF = new PIDFController(FollowerConstants.drivePIDFCoefficients);

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

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setStartingPose(Pose2d pose) {
        poseUpdater.setStartingPose(pose);
    }

    public void followPath(Path path) {
        isBusy = true;
        followingPathChain = false;
        currentPath = path;
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

    public void followPathChain(PathChain pathChain) {
        isBusy = true;
        followingPathChain = true;
        chainIndex = 0;
        currentPathChain = pathChain;
        currentPath = pathChain.getPath(chainIndex);
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

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
                // TODO: end detection
            }
        }

        drivePowers = driveVectorScaler.getDrivePowers(getCorrectiveVector(), getHeadingVector(), getDriveVector());

        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setPower(drivePowers[i]);
        }
    }

    public boolean isBusy() {
        return isBusy;
    }

    /**
     * This returns a Vector in the direction the robot must go to move along the path
     *
     * Note: This vector is clamped to be between [0, 1] in magnitude
     *
     * @return returns the drive vector
     */
    public Vector getDriveVector() {
        if (followingPathChain && chainIndex < currentPathChain.size()-1) {
            return new Vector(1, currentPath.getClosestPointHeadingGoal());
        }
        drivePIDF.updateError(currentPath.length() * currentPath.getClosestPointTValue() - getZeroPowerDistance());
        return new Vector(drivePIDF.runPIDF(), currentPath.getClosestPointHeadingGoal());
    }

    /**
     * This returns the distance the robot is projected to go when all power is cut from the drivetrain
     *
     * @return returns the projected distance
     */
    public double getZeroPowerDistance() {
        return -Math.pow(poseUpdater.getVelocity(), 2) / 2 * FollowerConstants.zeroPowerAcceleration;
    }

    /**
     * This returns a Vector in the direction of the robot that contains the heading correction
     * as its magnitude
     *
     * Note: This vector is clamped to between [0, 1] in magnitude
     *
     * @return returns the heading vector
     */
    public Vector getHeadingVector() {
        headingPIDF.updateError(MathFunctions.getSmallestAngleDifference(currentPath.getClosestPointHeadingGoal(), poseUpdater.getPose().getHeading()));
        return new Vector(MathFunctions.clamp(headingPIDF.runPIDF(), 0, 1), poseUpdater.getPose().getHeading());
    }

    /**
     * This returns a Vector in the direction the robot must go to account for both translational
     * error as well as centripetal force.
     *
     * Note: This vector is clamped to be between [0, 1] in magnitude
     *
     * @return returns the corrective vector
     */
    public Vector getCorrectiveVector() {
        return null;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only translational
     * error
     *
     * Note: This vector is clamped to be between [0, 1] in magnitude
     *
     * @return returns the translational vector
     */
    public Vector getTranslationalCorrection() {
        Vector translationalVector = new Vector(0,0);
        translationalPIDF.updateError(MathFunctions.distance(closestPose, poseUpdater.getPose()));
        translationalVector.setOrthogonalComponents(closestPose.getX() - poseUpdater.getPose().getX(), closestPose.getY() - poseUpdater.getPose().getY());
        translationalVector.setMagnitude(MathFunctions.clamp(translationalPIDF.runPIDF(), 0, 1));
        return translationalVector;
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
        return new Vector(MathFunctions.clamp(FollowerConstants.centrifugalScaling * FollowerConstants.mass * Math.pow(poseUpdater.getVelocity(), 2) * curvature,0,1), currentPath.getClosestPointHeadingGoal() + MathFunctions.getSign(curvature) * (Math.PI/2));
    }

    /**
     * IN A SOMEWHAT ORDERED LIST, HERE ARE THE TODOS
     */

    // TODO: make correctional vector
    // combine the translational and centripetal vectors

    // TODO: make centripetal force correction
    // do this by finding the curvature, then using the physics formula to find the force necessary and scaling that to a motor power


    // TODO: measure the deceleration of the drivetrain at 0 power and create the glide-into-position stopping system
    // do this by measuring if the distance from the drivetrain stopping is greater than the path end position
    // then, use a translational pid to move the projected end point onto the actual end point

    // TODO: alternate stopping mechanism?
    // still measure the deceleration, but now measure when the stopping distance is greater
    // than the path end position in terms of arc length
    // then, cut forward drive power, but keep correctional pids
    // alternatively, use a pid to slow down, so when stop distance exceeds arc length remaining,
    // correct backwards, and vice versa
}
