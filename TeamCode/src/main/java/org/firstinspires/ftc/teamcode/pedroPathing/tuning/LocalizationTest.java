package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

@TeleOp (group = "Pedro Pathing Tuning", name = "Localization Test")
public class LocalizationTest extends OpMode {
    private Follower follower;
    private Vector driveVector;
    private Vector headingVector;
    private Telemetry telemetryA;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryA.update();
    }

    @Override
    public void loop() {
        driveVector.setOrthogonalComponents(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
        driveVector.rotateVector(follower.getPose().getHeading());

        headingVector.setComponents(-gamepad1.left_stick_x, follower.getPose().getHeading());

        follower.setMovementVectors(new Vector(), headingVector, driveVector);
        follower.update();

        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.addData("total heading", follower.getTotalHeading());
        telemetryA.update();

    }
}
