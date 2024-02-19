package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.util.ArrayList;
import java.util.List;

public class PoseUpdater {
    private HardwareMap hardwareMap;

    private IMU imu;

    private ThreeWheelLocalizer localizer;

    private Pose2d startingPose = new Pose2d(0,0,0);

    private Pose2d previousPose = startingPose;

    private double xOffset = 0, yOffset = 0, headingOffset = 0;

    private long previousPoseTime, currentPoseTime;

    /**
     * Creates a new PoseUpdater from a hardware map
     *
     * @param hardwareMap the hardware map
     */
    public PoseUpdater(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        localizer = new ThreeWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
    }

    /**
     * Updates the robot's pose
     */
    public void update() {
        previousPose = applyOffset(localizer.getPoseEstimate());
        previousPoseTime = currentPoseTime;
        currentPoseTime = System.nanoTime();
        localizer.update();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param set the pose to set the starting pose to
     */
    public void setStartingPose(Pose2d set) {
        startingPose = set;
        previousPose = startingPose;
        previousPoseTime = System.nanoTime();
        currentPoseTime = System.nanoTime();
        localizer.setPoseEstimate(set);
    }

    /**
     *
     * This resets the current pose to a specified pose, using offsets to avoid having to reset using roadrunners pose reset
     *
     * @param set this is the pose we want to set the current pose to, using offsets
     */
    public void setCurrentPoseUsingOffset(Pose2d set) {
        Pose2d currentPose = localizer.getPoseEstimate();
        setXOffset(set.getX() - currentPose.getX());
        setYOffset(set.getY() - currentPose.getY());
        setHeadingOffset(MathFunctions.getTurnDirection(currentPose.getHeading(), set.getHeading()) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), set.getHeading()));
    }

    public void setXOffset(double offset) {
        xOffset = offset;
    }

    public void setYOffset(double offset) {
        yOffset = offset;
    }

    public void setHeadingOffset(double offset) {
        headingOffset = offset;
    }

    public double getXOffset() {
        return xOffset;
    }

    public double getYOffset() {
        return yOffset;
    }

    public double getHeadingOffset() {
        return headingOffset;
    }

    public Pose2d applyOffset(Pose2d pose) {
        return new Pose2d(pose.getX()+xOffset, pose.getY()+yOffset, pose.getHeading()+headingOffset);
    }

    public void resetOffset() {
        setXOffset(0);
        setYOffset(0);
        setHeadingOffset(0);
    }

    /**
     * This returns the current pose
     *
     * @return returns the current pose
     */
    public Pose2d getPose() {
        return applyOffset(localizer.getPoseEstimate());
    }

    /**
     * This sets the current pose
     *
     * @param set the pose to set the current pose to
     */
    public void setPose(Pose2d set) {
        resetOffset();
        localizer.setPoseEstimate(set);
    }

    /**
     * Returns the robot's pose from the previous update
     *
     * @return returns the robot's previous pose
     */
    public Pose2d getPreviousPose() {
        return previousPose;
    }

    /**
     * Returns the robot's change in pose from the previous update
     *
     * @return returns the robot's delta pose
     */
    public Pose2d getDeltaPose() {
        return getPose().minus(previousPose);
    }

    /**
     * This returns the velocity of the robot as a vector
     *
     * @return returns the velocity of the robot
     */
    public Vector getVelocity() {
        Vector velocity = new Vector();
        velocity.setOrthogonalComponents(getPose().getX() - previousPose.getX(), getPose().getY() - previousPose.getY());
        velocity.setMagnitude(MathFunctions.distance(getPose(), previousPose) / ((currentPoseTime-previousPoseTime)/Math.pow(10.0, 9)));
        return velocity;
    }

    /**
     * This returns the velocity of the robot as a vector
     *
     * @return returns the velocity of the robot
     */
    public double getAngularVelocity() {
        return MathFunctions.getTurnDirection(previousPose.getHeading(), getPose().getHeading()) * MathFunctions.getSmallestAngleDifference(getPose().getHeading(), previousPose.getHeading()) / ((currentPoseTime-previousPoseTime)/Math.pow(10.0, 9));
    }

    /**
     * This resets the heading of the robot to the IMU's heading
     */
    public void resetHeadingToIMU() {
        localizer.resetHeading(getNormalizedIMUHeading() + startingPose.getHeading());
    }

    /**
     * This returns the IMU heading normalized to be between [0, 2 PI] radians
     *
     * @return returns the normalized IMU heading
     */
    public double getNormalizedIMUHeading() {
        return MathFunctions.normalizeAngle(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
}
