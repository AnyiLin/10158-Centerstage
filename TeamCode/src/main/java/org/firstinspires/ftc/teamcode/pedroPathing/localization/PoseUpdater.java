package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

        // TODO: change this when new robot is built
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        localizer = new ThreeWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
    }

    /**
     * Updates the robot's pose
     */
    public void update() {
        previousPose = localizer.getPoseEstimate();
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
     * This returns the current pose
     *
     * @return returns the current pose
     */
    public Pose2d getPose() {
        return localizer.getPoseEstimate();
    }

    /**
     * This sets the current pose
     *
     * @param set the pose to set the current pose to
     */
    public void setPose(Pose2d set) {
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
