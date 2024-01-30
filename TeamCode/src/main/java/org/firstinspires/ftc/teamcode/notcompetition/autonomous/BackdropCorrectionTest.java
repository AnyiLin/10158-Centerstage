package org.firstinspires.ftc.teamcode.notcompetition.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "Color Sensor Test", group = "Autonomous Pathing Tuning")
public class BackdropCorrectionTest extends OpMode {
    private Telemetry telemetryA;

    public static double farBound = 130, closeBound = 140, correctionFactor = 0.1;

    private RevColorSensorV3 colorSensor;

    private Follower follower;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.holdPoint(new BezierPoint(new Point(0,0, Point.CARTESIAN)), 0);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

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
        double rawLightValue = colorSensor.getRawLightDetected();

        if (rawLightValue > closeBound) follower.poseUpdater.setXOffset(follower.poseUpdater.getXOffset()+correctionFactor*-1);
        if (rawLightValue < farBound) follower.poseUpdater.setXOffset(follower.poseUpdater.getXOffset()+correctionFactor);

        telemetryA.addData("raw light value", colorSensor.getRawLightDetected());
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
