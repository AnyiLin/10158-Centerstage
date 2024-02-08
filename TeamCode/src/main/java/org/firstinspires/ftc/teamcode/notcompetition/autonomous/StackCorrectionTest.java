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
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@Autonomous (name = "Distance Sensor Test", group = "Autonomous Pathing Tuning")
public class StackCorrectionTest extends OpMode {
    private Telemetry telemetryA;

    public static double correctionFactor = 3, deadZone = 0.85;

    private Timer distanceSensorDecimationTimer;

    private DistanceSensor leftDistanceSensor, rightDistanceSensor;

    private Follower follower;

    private Path forwards;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.holdPoint(new BezierPoint(new Point(0,0, Point.CARTESIAN)), 0);

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        distanceSensorDecimationTimer = new Timer();

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
    }

    @Override
    public void loop() {
        follower.update();

        stackCorrection(correctionFactor);

        telemetryA.addData("left", leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetryA.addData("right", rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetryA.addData("left connection info", leftDistanceSensor.getConnectionInfo());
        telemetryA.addData("right connection info", rightDistanceSensor.getConnectionInfo());
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData( "heading", follower.getPose().getHeading());
        telemetryA.update();
    }

    public void stackCorrection(double correctionPower) {
        if (distanceSensorDecimationTimer.getElapsedTime() > 20) {

            double error = leftDistanceSensor.getDistance(DistanceUnit.INCH) - rightDistanceSensor.getDistance(DistanceUnit.INCH);

            error *= -1;
            if (Math.abs(error) > deadZone) {
                follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() + distanceSensorDecimationTimer.getElapsedTimeSeconds() * correctionPower * MathFunctions.getSign(error));
            } else {
                //follower.poseUpdater.setYOffset(follower.getTranslationalError().getYComponent());
            }

            if (Math.abs(follower.poseUpdater.getYOffset()) > 6)
                follower.poseUpdater.setYOffset(6 * MathFunctions.getSign(follower.poseUpdater.getYOffset()));

            telemetry.addData("error", error);
            distanceSensorDecimationTimer.resetTimer();
        }
    }

    @Override
    public void stop() {
        super.stop();
    }
}
