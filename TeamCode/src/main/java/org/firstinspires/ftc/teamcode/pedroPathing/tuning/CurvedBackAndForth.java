package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "Curved Back And Forth", group = "Autonomous Pathing Tuning")
public class CurvedBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 20;

    private boolean forward = true;

    private Follower follower;

    private Path forwards, backwards;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        forwards = new Path(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN), new Point(DISTANCE,DISTANCE, Point.CARTESIAN)));
        backwards = new Path(new BezierCurve(new Point(DISTANCE,DISTANCE, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));

        backwards.setReversed(true);

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
        telemetryA.addData("Heading error", MathFunctions.getTurnDirection(follower.getPose().getHeading(), 0) * MathFunctions.getSmallestAngleDifference(0,follower.getPose().getHeading()));
        telemetryA.addData("isBusy", follower.isBusy());
        telemetryA.addData("heading", follower.getHeadingVector().getMagnitude());
        telemetryA.addData("corrective", follower.getCorrectiveVector().getMagnitude());
        telemetryA.addData("translational", follower.getTranslationalCorrection().getMagnitude());
        telemetryA.addData("centripetal", follower.getCentripetalForceCorrection().getMagnitude());
        telemetryA.addData("centripetal theta", follower.getCentripetalForceCorrection().getTheta());
        telemetryA.addData("drive", follower.getDriveVector().getTheta());
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("pose", follower.getClosestPose().getX() + ", " + follower.getClosestPose().getY());
        telemetryA.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
