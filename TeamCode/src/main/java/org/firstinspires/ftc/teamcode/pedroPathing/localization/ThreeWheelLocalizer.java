package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 * left on robot is y pos
 *
 * front of robot is x pos
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */

/**
 * This class is adapted from the Road Runner StandardTrackingWheelLocalizer class. Later, this will
 * be replaced with a custom localizer.
 */
@Config
public class ThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.37795; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double X_MULTIPLIER = 0.5008239963;
    public static double Y_MULTIPLIER = 0.5018874659;

    public static double leftX = -18.5/25.4 - 0.1, leftY = 164.4/25.4, rightX = -18.4/25.4 - 0.1, rightY = -159.6/25.4, strafeX = -107.9/25.4+0.25, strafeY = -1.1/25.4-0.23;

    private RoadRunnerEncoder leftEncoder, rightEncoder, strafeEncoder;

    private List<Integer> lastEncPositions, lastEncVels;

    public ThreeWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(leftX, leftY, 0), // left
                new Pose2d(rightX, rightY, 0), // right
                new Pose2d(strafeX, strafeY,  Math.toRadians(90)) // strafe
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        // TODO: redo the configs here
        leftEncoder = new RoadRunnerEncoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightEncoder = new RoadRunnerEncoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        strafeEncoder = new RoadRunnerEncoder(hardwareMap.get(DcMotorEx.class, "strafeEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder.setDirection(RoadRunnerEncoder.Direction.REVERSE);
        rightEncoder.setDirection(RoadRunnerEncoder.Direction.REVERSE);
        strafeEncoder.setDirection(RoadRunnerEncoder.Direction.FORWARD);
    }

    public void resetHeading(double heading) {
        setPoseEstimate(new Pose2d(getPoseEstimate().getX(), getPoseEstimate().getY(), heading));
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = strafeEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos) * X_MULTIPLIER,
                encoderTicksToInches(rightPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) strafeEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel) * X_MULTIPLIER,
                encoderTicksToInches(rightVel) * X_MULTIPLIER,
                encoderTicksToInches(frontVel) * Y_MULTIPLIER
        );
    }
}
