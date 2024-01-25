package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "Path Chain Test", group = "Autonomous Pathing Tuning")
public class PathChainTest extends OpMode {
    private Telemetry telemetryA;

    private Follower follower;

    private PathChain pathChain, pathChain2;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        follower.setStartingPose(new Pose2d(36+72, 72+50, Math.PI*1.5));

        pathChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(36+72, 72+50, Point.CARTESIAN), new Point(76.5, 106, Point.CARTESIAN), new Point(84, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .addPath(new BezierLine(new Point(84, 79, Point.CARTESIAN), new Point(85, 28, Point.CARTESIAN)))//, new Point(84, 28, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI*1.5)
                .setPathEndTimeout(0)
                .build();

        follower.followPath(pathChain);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("stuff");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        follower.update();

        telemetryA.addData("path number", follower.getCurrentPathNumber());
        telemetryA.addData("leftFront", follower.motorPowers()[0]);
        telemetryA.addData("leftRear", follower.motorPowers()[1]);
        telemetryA.addData("rightFront", follower.motorPowers()[2]);
        telemetryA.addData("rightRear", follower.motorPowers()[3]);
        telemetryA.addData("Heading error", MathFunctions.getTurnDirection(follower.getPose().getHeading(), 0) * MathFunctions.getSmallestAngleDifference(0,follower.getPose().getHeading()));
        telemetryA.addData("translational error heading", follower.asdf());
        telemetryA.addData("left side pathing power", follower.qwerty());
        telemetryA.addData("right side pathing power", follower.qwerty2());
        telemetryA.addData("isBusy", follower.isBusy());
        telemetryA.addData("heading", follower.getHeadingVector().getMagnitude());
        telemetryA.addData("corrective", follower.getCorrectiveVector().getMagnitude());
        telemetryA.addData("translational", follower.getTranslationalCorrection().getMagnitude());
        telemetryA.addData("centripetal", follower.getCentripetalForceCorrection().getMagnitude());
        telemetryA.addData("centripetal theta", follower.getCentripetalForceCorrection().getTheta());
        telemetryA.addData("drive", follower.getDriveVector().getTheta());
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("drive error", follower.zxcv());
        telemetryA.addData("pose", follower.getClosestPose().getX() + ", " + follower.getClosestPose().getY());
        telemetryA.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
