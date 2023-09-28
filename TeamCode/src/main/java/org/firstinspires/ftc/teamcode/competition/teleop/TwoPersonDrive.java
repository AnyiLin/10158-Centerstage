package org.firstinspires.ftc.teamcode.competition.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.RobotConstants;

@TeleOp(name = "Two Person Drive", group = "Drive")
public class TwoPersonDrive extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, liftMotor, intake;

    private Servo leftIntake, rightIntake, leftOuttake, rightOuttake, outerClaw, innerClaw;

    private final int
    FINE_ADJUST_LIFT_CHANGE = 100, // this is set to encoder ticks/second
    REGULAR_LIFT_CHANGE = 290, // this is set to encoder ticks/second
    LIFT_MAX = RobotConstants.LIFT_MAX,
    TOP_LINE_POSITION = RobotConstants.TOP_LINE_POSITION,
    MIDDLE_LINE_POSITION = RobotConstants.MIDDLE_LINE_POSITION,
    BOTTOM_LINE_POSITION = RobotConstants.BOTTOM_LINE_POSITION,
    INTAKE_VELOCITY = RobotConstants.INTAKE_VELOCITY;

    private final double
    INTAKE_CHANGE = 40, // this is set to degrees/second
            RIGHT_INTAKE_OFFSET = RobotConstants.RIGHT_INTAKE_OFFSET,
            LEFT_INTAKE_OUT_POSITION = RobotConstants.LEFT_INTAKE_OUT_POSITION,
            RIGHT_INTAKE_OUT_POSITION = RobotConstants.RIGHT_INTAKE_OUT_POSITION,
            LEFT_INTAKE_OUT_PAUSE_POSITION = RobotConstants.LEFT_INTAKE_OUT_PAUSE_POSITION,
            RIGHT_INTAKE_OUT_PAUSE_POSITION = RobotConstants.RIGHT_INTAKE_OUT_PAUSE_POSITION,
    LEFT_INTAKE_IN_POSITION = RobotConstants.LEFT_INTAKE_IN_POSITION,
    RIGHT_INTAKE_IN_POSITION = RobotConstants.RIGHT_INTAKE_IN_POSITION,
    LEFT_INTAKE_DROP_POSITION = RobotConstants.LEFT_INTAKE_DROP_POSITION,
    RIGHT_INTAKE_DROP_POSITION = RobotConstants.RIGHT_INTAKE_DROP_POSITION,
    LEFT_OUTTAKE_OUT_POSITION = RobotConstants.LEFT_OUTTAKE_OUT_POSITION,
    RIGHT_OUTTAKE_OUT_POSITION = RobotConstants.RIGHT_OUTTAKE_OUT_POSITION,
    LEFT_OUTTAKE_IN_POSITION= RobotConstants.LEFT_OUTTAKE_IN_POSITION,
    RIGHT_OUTTAKE_IN_POSITION = RobotConstants.RIGHT_OUTTAKE_IN_POSITION,
    LEFT_OUTTAKE_AVOID_POSITION = RobotConstants.LEFT_OUTTAKE_AVOID_POSITION,
    RIGHT_OUTTAKE_AVOID_POSITION = RobotConstants.RIGHT_OUTTAKE_AVOID_POSITION,
    OUTER_CLAW_CLOSE_POSITION = RobotConstants.OUTER_CLAW_CLOSE_POSITION,
    INNER_CLAW_CLOSE_POSITION = RobotConstants.INNER_CLAW_CLOSE_POSITION,
    OUTER_CLAW_OPEN_POSITION = RobotConstants.OUTER_CLAW_OPEN_POSITION,
    INNER_CLAW_OPEN_POSITION = RobotConstants.INNER_CLAW_OPEN_POSITION,
    INTAKE_SERVO_TO_DEGREES = RobotConstants.INTAKE_SERVO_TO_DEGREES,
    INTAKE_DEGREES_TO_SERVO = RobotConstants.INTAKE_DEGREES_TO_SERVO;

    private final long
    INTAKE_FULL_OUT_WAIT = RobotConstants.INTAKE_FULL_OUT_WAIT;

    private boolean outtakeIn, intakeGoingOut;

    private int liftTargetPosition;

    private long intakeOutStartTime, lastFrameTimeNano, deltaTimeNano;

    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setVelocity(0);

        liftMotor = leftLift; // set this to whatever is the lift motor with the encoder
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftTargetPosition = 0;
        liftMotor.setTargetPosition(liftTargetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (liftMotor.equals(leftLift)) {
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");
        leftOuttake = hardwareMap.get(Servo.class, "leftOuttake");
        rightOuttake = hardwareMap.get(Servo.class, "rightOuttake");
        outerClaw = hardwareMap.get(Servo.class, "outerClaw");
        innerClaw = hardwareMap.get(Servo.class, "innerClaw");

        leftIntake.setPosition(LEFT_INTAKE_IN_POSITION);
        rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
        leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
        rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
        outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
        innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);

        outtakeIn = true;
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
        lastFrameTimeNano = System.currentTimeMillis();
        while (opModeIsActive()) {

            updateFrameTime();

            double throttle = 0.2 + 0.8*gamepad1.right_trigger;

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = 0; // this is strafing

            if (gamepad2.right_stick_x!=0) {
                x = gamepad2.right_stick_x*0.3;
            } else if (gamepad1.left_bumper) {
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

            if (gamepad1.x || gamepad1.square) {
                intake.setVelocity(INTAKE_VELOCITY*gamepad1.left_trigger);
            } else if (gamepad1.b || gamepad1.circle) {
                intake.setVelocity(-INTAKE_VELOCITY*gamepad1.left_trigger);
            } else {
                intake.setVelocity(0);
            }

            if (gamepad1.y || gamepad1.triangle) {
                leftIntake.setPosition(LEFT_INTAKE_IN_POSITION);
                rightIntake.setPosition(RIGHT_INTAKE_IN_POSITION);
                intakeGoingOut = false;
            } else if (gamepad1.a || gamepad1.cross) {
                if (!intakeGoingOut) {
                    leftIntake.setPosition(LEFT_INTAKE_OUT_PAUSE_POSITION);
                    rightIntake.setPosition(RIGHT_INTAKE_OUT_PAUSE_POSITION);
                    intakeOutStartTime = System.currentTimeMillis();
                    intakeGoingOut = true;
                }
            }

            if (intakeGoingOut && (System.currentTimeMillis()-intakeOutStartTime > INTAKE_FULL_OUT_WAIT)) {
                leftIntake.setPosition(LEFT_INTAKE_OUT_POSITION);
                rightIntake.setPosition(RIGHT_INTAKE_OUT_POSITION);
                intakeGoingOut = false;
            }
            telemetry.addData("intake timer", System.currentTimeMillis()-intakeOutStartTime);
            telemetry.addData("intake out?", intakeGoingOut);
            telemetry.addData("system time millis", System.currentTimeMillis());
            telemetry.addData("intake start time", intakeOutStartTime);

            leftIntake.setPosition(leftIntake.getPosition()-gamepad1.right_stick_y*(deltaTimeNano /1000000000.0)*INTAKE_CHANGE*INTAKE_DEGREES_TO_SERVO);
            rightIntake.setPosition(1-leftIntake.getPosition()+RIGHT_INTAKE_OFFSET);
            if (leftIntake.getPosition() > LEFT_INTAKE_OUT_POSITION) {
                leftIntake.setPosition(LEFT_INTAKE_OUT_POSITION);
                rightIntake.setPosition(RIGHT_INTAKE_OUT_POSITION);
            }
            if (leftIntake.getPosition() < LEFT_INTAKE_DROP_POSITION) {
                leftIntake.setPosition(LEFT_INTAKE_DROP_POSITION);
                rightIntake.setPosition(RIGHT_INTAKE_DROP_POSITION);
            }

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
                liftTargetPosition += (gamepad2.right_trigger-gamepad2.left_trigger)*((deltaTimeNano /1000000000.0)*FINE_ADJUST_LIFT_CHANGE);
            } else {
                liftTargetPosition += (-gamepad2.left_stick_y)*((deltaTimeNano /1000000000.0)*REGULAR_LIFT_CHANGE);
            }
            telemetry.addData("lift target position", liftTargetPosition);
            telemetry.addData("gamepad2 left stick y", -gamepad2.left_stick_y);
            telemetry.addData("delta time nano", deltaTimeNano);

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

            updateLiftMotors();

            telemetry.update();
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

    public void updateLiftMotors() {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        updateLiftTargetPosition();
        if (liftMotor.equals(leftLift)) {
            rightLift.setPower(leftLift.getPower());
        } else {
            leftLift.setPower(rightLift.getPower());
        }
    }

    public void updateFrameTime() {
        deltaTimeNano = System.nanoTime()- lastFrameTimeNano;
        lastFrameTimeNano = System.nanoTime();
    }
}
