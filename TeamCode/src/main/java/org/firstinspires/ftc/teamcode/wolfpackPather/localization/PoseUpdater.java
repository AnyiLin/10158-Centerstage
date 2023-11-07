package org.firstinspires.ftc.teamcode.wolfpackPather.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.MathFunctions;

import java.util.ArrayList;
import java.util.List;

public class PoseUpdater {
    private HardwareMap hardwareMap;

    private IMU imu;

    private ThreeWheelLocalizer localizer;

    private Pose2d startingPose = new Pose2d(0,0,0);

    private Pose2d previousPose = startingPose;

    private long previousPoseTime, currentPoseTime;

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

    public void update() {
        previousPose = localizer.getPoseEstimate();
        previousPoseTime = currentPoseTime;
        currentPoseTime = System.nanoTime();
        localizer.update();
    }

    public void setStartingPose(Pose2d set) {
        startingPose = set;
        previousPose = startingPose;
        previousPoseTime = System.nanoTime();
        currentPoseTime = System.nanoTime();
    }

    public Pose2d getPose() {
        return localizer.getPoseEstimate();
    }

    public void setPose(Pose2d set) {
        localizer.setPoseEstimate(set);
    }

    public Pose2d getPreviousPose() {
        return previousPose;
    }

    public double getVelocity() {
        return MathFunctions.distance(getPose(), previousPose) / ((currentPoseTime-previousPoseTime)/Math.pow(10.0, 9));
    }

    public void resetHeadingToIMU() {
        localizer.resetHeading(getNormalizedIMUHeading() + startingPose.getHeading());
    }

    public double getNormalizedIMUHeading() {
        return MathFunctions.normalizeAngle(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
}
