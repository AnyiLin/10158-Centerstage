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
@Autonomous (name = "Strafe Velocity Tuner", group = "Autonomous Pathing Tuning")
public class StrafeVelocityTuner extends OpMode {
    private ArrayList<Double> velocities = new ArrayList<Double>();

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, leftExtension, rightExtension;
    private List<DcMotorEx> motors;

    private PoseUpdater poseUpdater;

    public static double DISTANCE = 40, RECORD_NUMBER = 10;

    private Telemetry telemetryA;

    private boolean end;

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

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(new Double(0));
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the right.");
        telemetryA.addLine("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetryA.addLine("Press cross or A to stop");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        leftFront.setPower(1);
        leftRear.setPower(-1);
        rightFront.setPower(-1);
        rightRear.setPower(1);
    }

    @Override
    public void loop() {
        if (gamepad1.cross || gamepad1.a) {
            requestOpModeStop();
        }

        poseUpdater.update();
        if (!end) {
            if (Math.abs(poseUpdater.getPose().getY()) > DISTANCE) {
                end = true;
                for (DcMotorEx motor : motors) {
                        motor.setPower(0);
                }
            } else {
                double currentVelocity = Math.abs(MathFunctions.dotProduct(poseUpdater.getVelocity(), new Vector(1, Math.PI/2)));
                velocities.add(new Double(currentVelocity));
                velocities.remove(0);
                /*
                if (currentVelocity < FollowerConstants.pathEndVelocity) {
                    end = true;
                }*/
            }
        } else {
            double average = 0;
            for (Double velocity : velocities) {
                average += velocity.doubleValue();
            }
            average /= (double) velocities.size();

            telemetryA.addData("strafe velocity:", average);
            telemetryA.update();
        }
    }

    @Override
    public void stop() {
        super.stop();
    }
}
