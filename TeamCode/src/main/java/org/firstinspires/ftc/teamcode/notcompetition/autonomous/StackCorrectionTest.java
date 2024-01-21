package org.firstinspires.ftc.teamcode.notcompetition.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "Distance Sensor Test", group = "Autonomous Pathing Tuning")
public class StackCorrectionTest extends OpMode {
    private Telemetry telemetryA;

    public static double correctionFactor = 0.2, deadZone = 1;

    private DistanceSensor leftDistanceSensor, rightDistanceSensor;

    private Follower follower;

    private Path forwards;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.holdPoint(new BezierPoint(new Point(0,0, Point.CARTESIAN)), 0);

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

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
        double error = rightDistanceSensor.getDistance(DistanceUnit.INCH)-leftDistanceSensor.getDistance(DistanceUnit.INCH);

        if (Math.abs(error) > deadZone) follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset()+correctionFactor*MathFunctions.getSign(error));

        telemetryA.addData("left", leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetryA.addData("right", rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData( "heading", follower.getPose().getHeading());
        telemetryA.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
