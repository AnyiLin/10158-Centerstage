package org.firstinspires.ftc.teamcode.pedroPathing.follower;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.drivePIDFSwitch;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.headingPIDFSwitch;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndHeading;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndTranslational;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.translationalPIDFSwitch;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Follower {
    private HardwareMap hardwareMap;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private List<DcMotorEx> motors;

    private DriveVectorScaler driveVectorScaler;

    private PoseUpdater poseUpdater;

    private Pose2d closestPose;

    private Path currentPath;

    private PathChain currentPathChain;

    private final int BEZIER_CURVE_BINARY_STEP_LIMIT = 9, DELTA_POSE_RECORD_LIMIT = 8;

    private int chainIndex;

    private boolean followingPathChain, isBusy, auto = true;

    private double[] drivePowers;

    private Vector[] teleOpMovementVectors = new Vector[] {new Vector(0,0), new Vector(0,0), new Vector(0,0)};

    private ArrayList<Pose2d> deltaPoses = new ArrayList<Pose2d>();
    private ArrayList<Pose2d> deltaDeltaPoses = new ArrayList<Pose2d>();

    private Pose2d averageDeltaPose, averagePreviousDeltaPose, averageDeltaDeltaPose;

    private PIDFController smallTranslationalXPIDF = new PIDFController(FollowerConstants.smallTranslationalPIDFCoefficients),
            smallTranslationalYPIDF = new PIDFController(FollowerConstants.smallTranslationalPIDFCoefficients),
            largeTranslationalXPIDF = new PIDFController(FollowerConstants.largeTranslationalPIDFCoefficients),
            largeTranslationalYPIDF = new PIDFController(FollowerConstants.largeTranslationalPIDFCoefficients),
            smallHeadingPIDF = new PIDFController(FollowerConstants.smallHeadingPIDFCoefficients),
            largeHeadingPIDF = new PIDFController(FollowerConstants.largeHeadingPIDFCoefficients),
            smallDrivePIDF = new PIDFController(FollowerConstants.smallDrivePIDFCoefficients),
            largeDrivePIDF = new PIDFController(FollowerConstants.largeDrivePIDFCoefficients);

    public static boolean useTranslational = true, useCentripetal = true, useHeading = true, useDrive = true;

    /**
     * This creates a new follower given a hardware map
     *
     * @param hardwareMap hardware map required
     */
    public Follower(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initialize();
    }

    /**
     * This creates a new follower given a hardware map and sets whether the follower is being used
     * in autonomous or teleop
     *
     * @param hardwareMap hardware map required
     * @param setAuto sets whether or not the follower is being used in autonomous or teleop
     */
    public Follower(HardwareMap hardwareMap, boolean setAuto) {
        this.hardwareMap = hardwareMap;
        setAuto(setAuto);
        initialize();
    }

    /**
     * This initializes the follower
     */
    public void initialize() {
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

        for (int i = 0; i < DELTA_POSE_RECORD_LIMIT; i++) {
            deltaPoses.add(new Pose2d(0,0,0));
        }
        for (int i = 0; i < DELTA_POSE_RECORD_LIMIT/2; i++) {
            deltaDeltaPoses.add(new Pose2d(0,0,0));
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
        if (auto) {
            closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
            if (currentPath.isAtParametricEnd()) {
                if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
                    // Not at last path, keep going
                    chainIndex++;
                    currentPath = currentPathChain.getPath(chainIndex);
                } else {
                    // At last path, run some end detection stuff
                    // set isBusy to false if at end
                    if (poseUpdater.getVelocity().getMagnitude() < currentPath.getPathEndVelocity() && MathFunctions.distance(poseUpdater.getPose(), closestPose) < currentPath.getPathEndTranslational() && MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) < currentPath.getPathEndHeading()) {
                        isBusy = false;
                        smallDrivePIDF.reset();
                        largeDrivePIDF.reset();
                        smallHeadingPIDF.reset();
                        largeHeadingPIDF.reset();
                        smallTranslationalXPIDF.reset();
                        smallTranslationalYPIDF.reset();
                        largeTranslationalXPIDF.reset();
                        largeTranslationalYPIDF.reset();
                    }
                }
            }

            if (isBusy) {
                drivePowers = driveVectorScaler.getDrivePowers(getCorrectiveVector(), getHeadingVector(), getDriveVector(), poseUpdater.getPose().getHeading());

                for (int i = 0; i < motors.size(); i++) {
                    motors.get(i).setPower(drivePowers[i]);
                }
            }
        } else {
            deltaPoses.add(poseUpdater.getDeltaPose());
            deltaPoses.remove(deltaPoses.get(deltaPoses.size() - 1));

            averageDeltaPose = new Pose2d(0,0,0);
            averagePreviousDeltaPose = new Pose2d(0,0,0);

            for (int i = 0; i < deltaPoses.size()/2; i++) {
                averageDeltaPose.plus(deltaPoses.get(i));
            }
            averageDeltaPose.div(deltaPoses.size()/2);

            for (int i = deltaPoses.size()/2; i < deltaPoses.size(); i++) {
                averagePreviousDeltaPose.plus(deltaPoses.get(i));
            }
            averagePreviousDeltaPose.div(deltaPoses.size()/2);

            deltaDeltaPoses.add(averageDeltaPose.minus(averagePreviousDeltaPose));
            deltaDeltaPoses.remove(deltaDeltaPoses.size() - 1);

            averageDeltaDeltaPose = new Pose2d(0,0,0);

            for (int i = 0; i < deltaDeltaPoses.size(); i++) {
                averageDeltaDeltaPose.plus(deltaDeltaPoses.get(i));
            }
            averageDeltaDeltaPose.div(deltaDeltaPoses.size());

            drivePowers = driveVectorScaler.getDrivePowers(teleOpMovementVectors[0], teleOpMovementVectors[1], teleOpMovementVectors[2], poseUpdater.getPose().getHeading());
        }
    }

    // TODO: REMOVE
    public double[] motorPowers() {
        return driveVectorScaler.getDrivePowers(getCorrectiveVector(), getHeadingVector(), getDriveVector(), poseUpdater.getPose().getHeading());
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
     * Sets the correctional, heading, and drive movement vectors for teleop enhancements
     */
    public void setMovementVectors(Vector correctional, Vector heading, Vector drive) {
        teleOpMovementVectors = new Vector[] {correctional, heading, drive};
    }

    /**
     * This returns a Vector in the direction the robot must go to move along the path
     *
     * Note: This vector is clamped to be at most 1 in magnitude
     *
     * @return returns the drive vector
     */
    public Vector getDriveVector() {
        if (!useDrive) return new Vector();
        if (followingPathChain && chainIndex < currentPathChain.size()-1) {
            return new Vector(1, currentPath.getClosestPointTangentVector().getTheta());
        }

        double driveError = getDriveVelocityGoal() - MathFunctions.dotProduct(poseUpdater.getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector()));

        if (Math.abs(driveError) < drivePIDFSwitch) {
            smallDrivePIDF.updateError(driveError);
            return new Vector(smallDrivePIDF.runPIDF(), currentPath.getClosestPointTangentVector().getTheta());
        }

        largeDrivePIDF.updateError(driveError);
        return new Vector(largeDrivePIDF.runPIDF(), currentPath.getClosestPointTangentVector().getTheta());
    }

    // TODO: remove
    public double zxcv() {
        return poseUpdater.getVelocity().getMagnitude();//getDriveVelocityGoal() - poseUpdater.getVelocity().getMagnitude();
    }

    /**
     * This returns the velocity the robot needs to be at to make it to the end of the trajectory
     *
     * @return returns the projected distance
     */
    public double getDriveVelocityGoal() {
        double distanceToGoal;
        if (!currentPath.isAtParametricEnd()) {
            distanceToGoal = currentPath.length() * (1 - currentPath.getClosestPointTValue());
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(getPose().getX() - currentPath.getLastControlPoint().getX(), getPose().getY() - currentPath.getLastControlPoint().getY());
            distanceToGoal = -MathFunctions.dotProduct(currentPath.getEndTangent(), offset);
        }
        return MathFunctions.getSign(distanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAcceleration() * distanceToGoal));
    }

    /**
     * This returns the distance the robot is projected to go when all power is cut from the drivetrain
     * as a vector
     *
     * @return returns the projected distance vector
     */
    public Vector getZeroPowerDistanceVector() {
        return new Vector(getZeroPowerDistance(), poseUpdater.getVelocity().getTheta());
    }

    /**
     * This returns the distance the robot is projected to go when all power is cut from the drivetrain
     *
     * @return returns the projected distance
     */
    public double getZeroPowerDistance() {
        return -Math.pow(poseUpdater.getVelocity().getMagnitude(), 2) / (2 * currentPath.getZeroPowerAcceleration());
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
        if (!useHeading) return new Vector();
        double headingError = MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) * MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal());
        if (Math.abs(headingError) < headingPIDFSwitch) {
            smallHeadingPIDF.updateError(headingError);
            return new Vector(MathFunctions.clamp(smallHeadingPIDF.runPIDF(), -1, 1), poseUpdater.getPose().getHeading());
        }
        largeHeadingPIDF.updateError(headingError);
        return new Vector(MathFunctions.clamp(largeHeadingPIDF.runPIDF(), -1, 1), poseUpdater.getPose().getHeading());
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
        if (!useTranslational) return new Vector();
        Vector translationalVector = new Vector();
        double x = Math.abs(closestPose.getX() - poseUpdater.getPose().getX());
        if (closestPose.getX() < poseUpdater.getPose().getX()) x *= -1;
        double y = Math.abs(closestPose.getY() - poseUpdater.getPose().getY());
        if (closestPose.getY() < poseUpdater.getPose().getY()) y *= -1;
        if (MathFunctions.distance(poseUpdater.getPose(), closestPose) < translationalPIDFSwitch) {
            smallTranslationalXPIDF.updateError(x);
            smallTranslationalYPIDF.updateError(y);
            translationalVector.setOrthogonalComponents(smallTranslationalXPIDF.runPIDF(), smallTranslationalYPIDF.runPIDF());
        } else {
            largeTranslationalXPIDF.updateError(x);
            largeTranslationalYPIDF.updateError(y);
            translationalVector.setOrthogonalComponents(largeTranslationalXPIDF.runPIDF(), largeTranslationalYPIDF.runPIDF());
        }
        translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), 0, 1));
        // TODO: fix
        return MathFunctions.subtractVectors(translationalVector, new Vector(MathFunctions.dotProduct(translationalVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));
    }

    // TODO: remove later
    public double asdf() {
        return getTranslationalCorrection().getTheta();
    }

    public double qwerty() {
        return driveVectorScaler.getLeftSidePath().getTheta();
    }

    public double qwerty2() {
        return driveVectorScaler.getRightSidePath().getTheta();
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
        if (!useCentripetal) return new Vector();
        double curvature;
        if (auto) {
            curvature = currentPath.getClosestPointCurvature();
        } else {
            double yPrime = averageDeltaPose.getY() / averageDeltaPose.getX();
            double yDoublePrime = averageDeltaDeltaPose.getY() / averageDeltaPose.getX();
            curvature = (Math.pow(Math.sqrt(1 + Math.pow(yPrime, 2)), 3)) / (yDoublePrime);
        }
        if (Double.isNaN(curvature)) return new Vector();
        return new Vector(MathFunctions.clamp(FollowerConstants.centrifugalScaling * FollowerConstants.mass * Math.pow(MathFunctions.dotProduct(poseUpdater.getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 2) * curvature,-1,1), currentPath.getClosestPointHeadingGoal() + MathFunctions.getSign(curvature) * (Math.PI/2));
    }

    /**
     * This returns the closest pose to the robot on the path the follower is currently following
     *
     * @return returns the closest pose
     */
    public Pose2d getClosestPose() {
        return closestPose;
    }

    /**
     * This sets whether or not the follower is being used in auto or teleop
     *
     * @param set sets auto or not
     */
    public void setAuto(boolean set) {
        auto = set;
    }
}
