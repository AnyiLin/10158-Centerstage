package org.firstinspires.ftc.teamcode.competition.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.util.RobotConstants;

@TeleOp(name = "Two Person Drive", group = "Drive")
public class TwoPersonDrive extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, liftMotor;

    private final int LIFT_VELOCITY = RobotConstants.LIFT_VELOCITY,
    LIFT_MAX = RobotConstants.LIFT_MAX,
    FINE_ADJUST_LIFT_CHANGE = 100, // this is set to encoder ticks/second
    REGULAR_LIFT_CHANGE = 290; // this is set to encoder ticks/second

    private int liftTargetPosition;

    private float lastFrameTimeMillis, deltaTimeMillis;

    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        liftMotor = leftLift; // set this to whatever is the lift motor with the encoder

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftTargetPosition = 0;
        liftMotor.setTargetPosition(liftTargetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setVelocity(LIFT_VELOCITY);
        if (liftMotor.equals(leftLift)) {
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * IMPORTANT NOTES:
     *
     * For the Double Shock controllers:
     * triangle - top
     * circle - right
     * cross (X-shaped) - bottom
     * square - left
     *
     * For the older Logitech controllers:
     * Y - top
     * B - right
     * A - bottom
     * X - left
     */

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        lastFrameTimeMillis = System.currentTimeMillis();
        while (opModeIsActive()) {

            updateFrameTime();

            double throttle = 0.2 + 0.8*gamepad1.right_trigger;

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = 0; // this is strafing

            if (gamepad1.left_bumper) {
                x = -1;
            } else if (gamepad1.right_bumper) {
                x = 1;
            }

            double rx = 0;
            if (Math.abs(gamepad1.left_stick_x)>0.1) rx = gamepad1.left_stick_x;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1)/throttle;
            double leftFrontPower = (y + x + rx) / denominator;
            double leftRearPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightRearPower = (y + x - rx) / denominator;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            // TODO: gamepad 1 left trigger controls intake

            // TODO: some button on gamepad1 will also control intake virtual four bar

            // TODO: make intake and claw not collide

            /*
            any changes to slides is a change in the target position of pid
            left stick controls slides
            d pad is presets
            left trigger is fine adjust slides down
            right trigger is fine adjust slides up
            right bumper releases claw closest to backdrop
            left bumper releases claw farther from backdrop
            right stick is fine adjust strafing only

            slides presets:
            dpad_up - top line
            dpad_left - middle line
            dpad_down - bottom line
            dpad_right - reset
            */

            // TODO: implement literally everything else, including lift passive pid control

            if (gamepad2.left_trigger>0||gamepad2.right_trigger>0) {
                liftTargetPosition += (gamepad2.right_trigger-gamepad2.left_trigger)*((deltaTimeMillis/1000.0)*FINE_ADJUST_LIFT_CHANGE);
            } else {
                liftTargetPosition += (-gamepad2.left_stick_y)*((deltaTimeMillis/1000.0)*REGULAR_LIFT_CHANGE);
            }

            if (gamepad2.dpad_up) {
                // TODO: top line preset
            }
            if (gamepad2.dpad_left) {
                // TODO: middle line preset
            }
            if (gamepad2.dpad_down) {
                // TODO: bottom line preset
            }
            if (gamepad2.dpad_right) {
                // TODO: reset preset
            }

            updateLiftTargetPosition();
        }
    }

    public void startPreset(int liftPosition) {
        liftTargetPosition = liftPosition;
    }

    public void updateLiftTargetPosition() {
        if (liftTargetPosition < 0) liftTargetPosition = 0;
        if (liftTargetPosition > LIFT_MAX) liftTargetPosition = LIFT_MAX;
        liftMotor.setTargetPosition(liftTargetPosition);
    }

    public void updateFrameTime() {
        deltaTimeMillis = System.currentTimeMillis()-lastFrameTimeMillis;
        lastFrameTimeMillis = System.currentTimeMillis();
    }
}
