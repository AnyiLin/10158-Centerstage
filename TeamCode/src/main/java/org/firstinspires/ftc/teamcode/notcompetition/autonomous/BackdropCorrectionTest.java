package org.firstinspires.ftc.teamcode.notcompetition.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Config
@Autonomous (name = "Color Sensor Test", group = "Autonomous Pathing Tuning")
public class BackdropCorrectionTest extends OpMode {
    private Telemetry telemetryA;

    private Timer distanceSensorDecimationTimer, opModeTimer;

    private DistanceSensor rearDistanceSensor;

    private Follower follower;

    private Point goalPoint;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        goalPoint = new Point(0,0, Point.CARTESIAN);
        follower.holdPoint(new BezierPoint(goalPoint), 0);

        rearDistanceSensor = hardwareMap.get(DistanceSensor.class, "rearDistanceSensor");

        distanceSensorDecimationTimer = new Timer();
        opModeTimer = new Timer();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("stuff");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        distanceSensorDecimationTimer.resetTimer();
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        backdropCorrection(new Pose2d(0,0), 2);

        telemetryA.addData("distance mm", rearDistanceSensor.getDistance(DistanceUnit.MM));
        telemetryA.addData("distance in", rearDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData( "heading", follower.getPose().getHeading());
        telemetryA.update();
    }
    public void backdropCorrection(Pose2d scorePose, double distanceGoal) {
        if (distanceSensorDecimationTimer.getElapsedTime() > 100) {

            double distance = rearDistanceSensor.getDistance(DistanceUnit.MM);

            if (distance != 65535) {
                //follower.holdPoint(new BezierPoint(new Point(MathFunctions.clamp(follower.getPose().getX() - ((distance / 25.4) - distanceGoal), scorePose.getX() - 4, scorePose.getX() + 4), scorePose.getY(), Point.CARTESIAN)), 0);
                goalPoint.setCoordinates(MathFunctions.clamp(follower.getPose().getX() - ((distance / 25.4) - distanceGoal), scorePose.getX() - 4, scorePose.getX() + 4),goalPoint.getY(),Point.CARTESIAN);
            }
/*
            // too close
            if (distance < 0.5)
                follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() + distanceSensorDecimationTimer.getElapsedTimeSeconds() * 1.5);

            // too far
            if (distance > 0.75)
                follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() - distanceSensorDecimationTimer.getElapsedTimeSeconds() * 1.5);

            // to do add some sort of deadzone or dampening
            // perhaps take note of the estimated pose at the start and see how far off we need to go instead of incrementing off of the current one
            // or just remove the getyoffset thing? think about later
            follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() - (distance - 2));

            if (Math.abs(follower.poseUpdater.getYOffset()) > 1.5)
                follower.poseUpdater.setYOffset(1.5 * MathFunctions.getSign(follower.poseUpdater.getYOffset()));
*/
            //telemetry.addData("rear distance value", distance);
            distanceSensorDecimationTimer.resetTimer();
        }
    }

    @Override
    public void stop() {
        super.stop();
    }
}
