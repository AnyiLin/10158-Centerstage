package org.firstinspires.ftc.teamcode.notcompetition.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "Turn Test", group = "Autonomous Pathing Tuning")
public class TurnTest extends OpMode {
    private Telemetry telemetryA;

    private Follower follower;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(10,0, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(0,Math.PI*1.5)
                .setPathEndTimeout(1000)
                .build();
        follower.followPath(path);

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

        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData( "heading", follower.getPose().getHeading());
        telemetryA.addData( "turn direction", MathFunctions.getTurnDirection(0, Math.PI * 1.5));
        telemetryA.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
