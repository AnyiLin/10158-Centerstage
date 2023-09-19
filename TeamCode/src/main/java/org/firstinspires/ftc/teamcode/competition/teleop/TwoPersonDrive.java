package org.firstinspires.ftc.teamcode.competition.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.RobotConstants;

@TeleOp(name = "Two Person Drive", group = "Drive")
public class TwoPersonDrive extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, leftLift, rightLift;

    private boolean liftInMotion;

    private int lastLiftPosition;

    private long liftStartTime;

    private final int LIFT_VELOCITY = RobotConstants.LIFT_VELOCITY;

    private final long LIFT_TIME_OUT = 1000;

    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
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
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = 0; // this is strafing

            if (gamepad1.right_trigger != 0 || gamepad1.left_trigger != 0) {
                x = (-gamepad1.right_trigger+gamepad1.left_trigger);
            } else if (gamepad1.left_bumper) {
                x = 0.25;
            } else if (gamepad1.right_bumper) {
                x = -0.25;
            }

            double rx = 0;
            if (Math.abs(gamepad1.left_stick_x)>0.1) rx = -gamepad1.left_stick_x;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftRearPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightRearPower = (y + x - rx) / denominator;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            /*
            slides presets:
            dpad_up - top line
            dpad_left - middle line
            dpad_down - bottom line
            dpad_right - reset
            */

            // TODO: maybe add force override of liftInMotion

            if (liftInMotion) {
                if (!leftLift.isBusy()&&!rightLift.isBusy()) {
                    leftLift.setPower(0);
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setPower(0);
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastLiftPosition = leftLift.getCurrentPosition();
                    liftInMotion = false;
                }
                if (System.currentTimeMillis()-liftStartTime>LIFT_TIME_OUT) {
                    leftLift.setPower(0);
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setPower(0);
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastLiftPosition = leftLift.getCurrentPosition();
                    liftInMotion = false;
                }
            } else {
                // TODO: implement literally everything else, including lift passive pid control

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
                    // TODO: reset line preset
                }
            }
        }
    }

    public void startPreset(int liftPosition) {
        liftInMotion = true;
        leftLift.setTargetPosition(liftPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setVelocity(LIFT_VELOCITY);
        rightLift.setTargetPosition(liftPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setVelocity(LIFT_VELOCITY);
        liftStartTime = System.currentTimeMillis();
    }
}
