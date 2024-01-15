package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "Straight Back And Forth", group = "Autonomous Pathing Tuning")
public class StraightBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private Path forwards, backwards;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

        follower.followPath(forwards);

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
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryA.addData("forward", forward);
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
        telemetryA.addData("drive", follower.getDriveVector().getTheta());
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("drive error", follower.zxcv());
        telemetryA.addData("pose", follower.getClosestPose().getX() + ", " + follower.getClosestPose().getY());
        double[] motorPowers = follower.motorPowers();
        for (int i = 0; i < motorPowers.length; i++) {
            telemetryA.addData("motor " + i, motorPowers[i]);
        }
        telemetryA.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
