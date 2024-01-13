package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@Autonomous (name = "Forward Zero Power Acceleration Tuner", group = "Autonomous Pathing Tuning")
public class ForwardZeroPowerAccelerationTuner extends OpMode {
    private ArrayList<Double> accelerations = new ArrayList<Double>();

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private List<DcMotorEx> motors;

    private PoseUpdater poseUpdater;

    public static double VELOCITY = 10;

    private double previousVelocity;

    private long previousTimeNano;

    private Telemetry telemetryA;

    private boolean stopping, end;

    @Override
    public void init() {
        poseUpdater = new PoseUpdater(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("The robot will run forward until it reaches " + VELOCITY + " inches per second");
        telemetryA.addLine("Make sure you have enough room");
        telemetryA.addLine("Press cross or A to stop");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        for (DcMotorEx motor : motors) {
            motor.setPower(1);
        }
    }

    @Override
    public void loop() {
        if (gamepad1.cross || gamepad1.a) {
            requestOpModeStop();
        }

        poseUpdater.update();
        Vector heading = new Vector(1.0, poseUpdater.getPose().getHeading());
        if (!end) {
            if (!stopping) {
                if (MathFunctions.dotProduct(poseUpdater.getVelocity(), heading) > VELOCITY) {
                    previousVelocity = MathFunctions.dotProduct(poseUpdater.getVelocity(), heading);
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    for (DcMotorEx motor : motors) {
                        motor.setPower(0);
                    }
                }
            } else {
                double currentVelocity = MathFunctions.dotProduct(poseUpdater.getVelocity(), heading);
                accelerations.add(new Double((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9))));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < FollowerConstants.pathEndVelocity) {
                    end = true;
                }
            }
        } else {
            double average = 0;
            for (Double acceleration : accelerations) {
                average += acceleration.doubleValue();
            }
            average /= (double)accelerations.size();

            telemetryA.addData("average acceleration:", average);
            telemetryA.update();
        }
    }

    @Override
    public void stop() {
        super.stop();
    }
}
