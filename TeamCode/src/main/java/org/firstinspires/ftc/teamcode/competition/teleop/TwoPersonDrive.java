package org.firstinspires.ftc.teamcode.competition.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

@TeleOp(name = "Two Person Drive", group = "Drive")
public class TwoPersonDrive extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, intake;

    private Servo leftIntake, rightIntake, leftOuttake, rightOuttake, outerClaw, innerClaw;

    private final int
            LIFT_VELOCITY = RobotConstants.LIFT_VELOCITY,
            LIFT_VELOCITY_TOLERANCE = RobotConstants.LIFT_VELOCITY_TOLERANCE,
            FINE_ADJUST_LIFT_CHANGE = 300, // this is set to encoder ticks/second
            REGULAR_LIFT_CHANGE = 2*620, // this is set to encoder ticks/second
            LIFT_MAX = RobotConstants.LIFT_MAX,
            LIFT_TOLERANCE = RobotConstants.LIFT_TOLERANCE,
            TOP_LINE_POSITION = RobotConstants.TOP_LINE_POSITION,
            MIDDLE_LINE_POSITION = RobotConstants.MIDDLE_LINE_POSITION,
            BOTTOM_LINE_POSITION = RobotConstants.BOTTOM_LINE_POSITION,
            LIFT_GRAB_POSITION = RobotConstants.LIFT_GRAB_POSITION,
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
            LEFT_INTAKE_MIDDLE_POSITION = RobotConstants.LEFT_INTAKE_MIDDLE_POSITION,
            RIGHT_INTAKE_MIDDLE_POSITION = RobotConstants.RIGHT_INTAKE_MIDDLE_POSITION,
            LEFT_INTAKE_DROP_POSITION = RobotConstants.LEFT_INTAKE_DROP_POSITION,
            RIGHT_INTAKE_DROP_POSITION = RobotConstants.RIGHT_INTAKE_DROP_POSITION,
            RIGHT_OUTTAKE_OFFSET = RobotConstants.RIGHT_OUTTAKE_OFFSET,
            LEFT_OUTTAKE_OUT_POSITION = RobotConstants.LEFT_OUTTAKE_OUT_POSITION,
            RIGHT_OUTTAKE_OUT_POSITION = RobotConstants.RIGHT_OUTTAKE_OUT_POSITION,
            LEFT_OUTTAKE_IN_POSITION= RobotConstants.LEFT_OUTTAKE_IN_POSITION,
            RIGHT_OUTTAKE_IN_POSITION = RobotConstants.RIGHT_OUTTAKE_IN_POSITION,
            LEFT_OUTTAKE_GRAB_POSITION = RobotConstants.LEFT_OUTTAKE_GRAB_POSITION,
            RIGHT_OUTTAKE_GRAB_POSITION = RobotConstants.RIGHT_OUTTAKE_GRAB_POSITION,
            LEFT_OUTTAKE_AVOID_POSITION = RobotConstants.LEFT_OUTTAKE_AVOID_POSITION,
            RIGHT_OUTTAKE_AVOID_POSITION = RobotConstants.RIGHT_OUTTAKE_AVOID_POSITION,
            OUTER_CLAW_CLOSE_POSITION = RobotConstants.OUTER_CLAW_CLOSE_POSITION,
            INNER_CLAW_CLOSE_POSITION = RobotConstants.INNER_CLAW_CLOSE_POSITION,
            OUTER_CLAW_OPEN_POSITION = RobotConstants.OUTER_CLAW_OPEN_POSITION,
            INNER_CLAW_OPEN_POSITION = RobotConstants.INNER_CLAW_OPEN_POSITION,
            INTAKE_SERVO_TO_DEGREES = RobotConstants.INTAKE_SERVO_TO_DEGREES,
            INTAKE_DEGREES_TO_SERVO = RobotConstants.INTAKE_DEGREES_TO_SERVO;

    private final long
            INTAKE_IN_WAIT = RobotConstants.INTAKE_IN_WAIT,
            INTAKE_OBSTACLE_OUT_WAIT = RobotConstants.INTAKE_OBSTACLE_OUT_WAIT,
            INTAKE_OBSTACLE_OUT_RETRACT_WAIT = RobotConstants.INTAKE_OBSTACLE_OUT_RETRACT_WAIT,
            INTAKE_OBSTACLE_IN_WAIT = RobotConstants.INTAKE_OBSTACLE_IN_WAIT,
            OUTTAKE_OBSTACLE_FOLD_IN_WAIT = RobotConstants.OUTTAKE_OBSTACLE_FOLD_IN_WAIT,
            CLAW_GRAB_WAIT = RobotConstants.CLAW_GRAB_WAIT,
            CLAW_ACTUALLY_GRAB_WAIT = RobotConstants.CLAW_ACTUALLY_GRAB_WAIT,
            CLAW_CLOSE_WAIT = RobotConstants.CLAW_CLOSE_WAIT,
            CLAW_LIFT_WAIT = RobotConstants.CLAW_LIFT_WAIT,
            PRESET_TIMEOUT = RobotConstants.PRESET_TIMEOUT,
            RESET_PIXEL_DROP_WAIT = RobotConstants.RESET_PIXEL_DROP_WAIT,
            RESET_FOLD_IN_WAIT = RobotConstants.RESET_FOLD_IN_WAIT,
            INTAKE_FULL_OUT_WAIT = RobotConstants.INTAKE_FULL_OUT_WAIT;

    private boolean resetFoldIn, resetPixelDrop, liftInGrabbingPosition, resetInMotion, presetLifting, clawLifting, clawClosing, clawActuallyGrab, outtakeIn, intakeIn, intakeGoingOut, intakeGoingInObstacle, intakeGoingInObstacleFoldUp, intakeGoingIn, intakeGoingOutObstacle, intakeGoingOutObstacleRetract, clawGrabbing, presetInMotion, presetQueue;

    private int liftTargetPosition, rightLiftError, presetTargetPosition;

    private long resetFoldInStartTime, resetPixelDropStartTime, presetStartTime, clawGrabbingStartTime, intakeOutStartTime, intakeInObstacleStartTime, intakeInStartTime, intakeOutObstacleStartTime, lastFrameTimeNano, deltaTimeNano;

    private Telemetry telemetryA;

    public void initialize() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

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

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftTargetPosition = 0;
        rightLiftError = 0;
        leftLift.setTargetPosition(liftTargetPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setTargetPosition(liftTargetPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");
        leftOuttake = hardwareMap.get(Servo.class, "leftOuttake");
        rightOuttake = hardwareMap.get(Servo.class, "rightOuttake");
        outerClaw = hardwareMap.get(Servo.class, "outerClaw");
        innerClaw = hardwareMap.get(Servo.class, "innerClaw");

        leftIntake.setPosition(LEFT_INTAKE_DROP_POSITION);
        rightIntake.setPosition(RIGHT_INTAKE_DROP_POSITION);
        leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
        rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
        outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
        innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);

        outtakeIn = true;
        intakeIn = true;
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

            if (!presetInMotion && !resetInMotion) {
                if (gamepad1.x || gamepad1.square) {
                    intake.setVelocity(INTAKE_VELOCITY * gamepad1.left_trigger);
                } else if (gamepad1.b || gamepad1.circle) {
                    intake.setVelocity(-INTAKE_VELOCITY * gamepad1.left_trigger);
                } else {
                    intake.setVelocity(0);
                }

                if (gamepad1.y || gamepad1.triangle) {
                    if (!intakeIn) {
                        intakeIn = true;
                        if (outtakeIn) {
                            if (!intakeGoingInObstacle && !intakeGoingInObstacleFoldUp) {
                                leftIntake.setPosition(LEFT_INTAKE_MIDDLE_POSITION);
                                rightIntake.setPosition(RIGHT_INTAKE_MIDDLE_POSITION);
                                intakeInObstacleStartTime = System.currentTimeMillis();
                                intakeGoingInObstacle = true;
                            }
                        } else {
                            if (!intakeGoingIn) {
                                leftIntake.setPosition(LEFT_INTAKE_IN_POSITION);
                                rightIntake.setPosition(RIGHT_INTAKE_IN_POSITION);
                                intakeGoingIn = true;
                                intakeInStartTime = System.currentTimeMillis();
                            }
                        }
                        intakeGoingOut = false;
                        intakeGoingOutObstacle = false;
                        intakeGoingOutObstacleRetract = false;
                    }
                } else if (gamepad1.a || gamepad1.cross) {
                    if (intakeIn) {
                        intakeIn = false;
                        if (outtakeIn) {
                            if (!intakeGoingOutObstacle && !intakeGoingOutObstacleRetract) {
                                leftOuttake.setPosition(LEFT_OUTTAKE_AVOID_POSITION);
                                rightOuttake.setPosition(RIGHT_OUTTAKE_AVOID_POSITION);
                                intakeGoingOutObstacle = true;
                                intakeOutObstacleStartTime = System.currentTimeMillis();
                            }
                        } else {
                            if (!intakeGoingOut) {
                                leftIntake.setPosition(LEFT_INTAKE_OUT_PAUSE_POSITION);
                                rightIntake.setPosition(RIGHT_INTAKE_OUT_PAUSE_POSITION);
                                intakeOutStartTime = System.currentTimeMillis();
                                intakeGoingOut = true;
                            }
                        }
                        intakeGoingInObstacle = false;
                        intakeGoingInObstacleFoldUp = false;
                        intakeGoingIn = false;
                    } else {
                        if (!intakeGoingOutObstacle && !intakeGoingOutObstacleRetract && !intakeGoingOut) {
                            leftIntake.setPosition(LEFT_INTAKE_OUT_POSITION);
                            rightIntake.setPosition(RIGHT_INTAKE_OUT_POSITION);
                        }
                    }
                }
            }

            if (!presetInMotion && !resetInMotion) fineAdjustIntakeV4B();

            /*
            any changes to slides is a change in the target position of pid
            left stick controls slides
            d pad is presets
            left trigger is fine adjust slides down
            right trigger is fine adjust slides up
            right bumper releases claw closest to backdrop
            left bumper releases claw farther from backdrop
            right stick is fine adjust strafing on the x and fine adjust outtake on the y

            slides presets:
            dpad_up - top line
            dpad_left - middle line
            dpad_down - bottom line
            dpad_right - reset
            */

            if (!presetInMotion && !resetInMotion) {
                if (gamepad2.right_bumper) outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
                if (gamepad2.left_bumper) innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);
            }

            if (!presetInMotion && !resetInMotion) {
                if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
                    liftTargetPosition += (gamepad2.right_trigger - gamepad2.left_trigger) * ((deltaTimeNano / 1000000000.0) * FINE_ADJUST_LIFT_CHANGE);
                } else {
                    liftTargetPosition += (-gamepad2.left_stick_y) * ((deltaTimeNano / 1000000000.0) * REGULAR_LIFT_CHANGE);
                }
            }

            if (!resetInMotion && !resetInMotion) {
                if (gamepad2.dpad_up) {
                    startPreset(TOP_LINE_POSITION);
                }
                if (gamepad2.dpad_left) {
                    startPreset(MIDDLE_LINE_POSITION);
                }
                if (gamepad2.dpad_down) {
                    startPreset(BOTTOM_LINE_POSITION);
                }
                if (gamepad2.dpad_right) {
                    resetPreset();
                }
            }

            asyncTimers();

            if (presetInMotion) detectPresetEnd();

            if (!presetInMotion && !resetInMotion) updateLiftMotors();


            telemetryA.addData("lift target position", liftTargetPosition);
            telemetryA.addData("left lift current position", leftLift.getCurrentPosition());
            telemetryA.addData("left lift target position", leftLift.getTargetPosition());
            telemetryA.addData("right lift current position", rightLift.getCurrentPosition());
            telemetryA.addData("right lift target position", rightLift.getTargetPosition());
            telemetryA.addData("delta time nano", deltaTimeNano);
            telemetryA.update();
        }
    }

    public void startPreset(int liftPosition) {
        if (!presetInMotion) presetStartTime = System.currentTimeMillis();
        presetInMotion = true;
        presetTargetPosition = liftPosition;
        if (outtakeIn) { // if the outtake is in, grab and go
            if (!intakeIn) {
                intakeIn = true;
                if (!intakeGoingInObstacle && !intakeGoingInObstacleFoldUp) {
                    leftIntake.setPosition(LEFT_INTAKE_MIDDLE_POSITION);
                    rightIntake.setPosition(RIGHT_INTAKE_MIDDLE_POSITION);
                    intakeInObstacleStartTime = System.currentTimeMillis();
                    intakeGoingInObstacle = true;
                    presetQueue = true;
                } else {
                    presetQueue = true;
                }
                intakeGoingOut = false;
                intakeGoingOutObstacle = false;
                intakeGoingOutObstacleRetract = false;
            } else {
                outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
                innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);
                outtakeIn = false;
                liftTargetPosition = LIFT_GRAB_POSITION;
                updateLiftMotors();
                liftInGrabbingPosition = false;
            }
        } else { // if the outtake isnt in, then just go to the position specified
            if (!liftInGrabbingPosition && !clawGrabbing && !clawActuallyGrab && !clawClosing && !clawLifting) {
                presetLifting = true;
                liftTargetPosition = presetTargetPosition;
                updateLiftMotors();
            }
        }
    }

    public void resetPreset() {
        // TODO: make reset
        resetInMotion = true;

        presetLifting = false;
        presetInMotion = false;
        intakeGoingInObstacle = false;
        intakeGoingInObstacleFoldUp = false;
        intakeGoingIn = false;
        intakeGoingOut = false;
        intakeGoingOutObstacle = false;
        intakeGoingOutObstacleRetract = false;
        clawGrabbing = false;
        clawActuallyGrab = false;
        clawClosing = false;
        clawLifting = false;

        if (!outtakeIn) {
            innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);
            outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
        }

        resetPixelDrop = true;
        resetPixelDropStartTime = System.currentTimeMillis();
    }

    public void detectPresetEnd() {
        if (presetLifting && ((leftLift.getCurrentPosition()<=liftTargetPosition+LIFT_TOLERANCE&&leftLift.getCurrentPosition()>=liftTargetPosition-LIFT_TOLERANCE)&&leftLift.getVelocity()<LIFT_VELOCITY_TOLERANCE)) {
            presetInMotion = false;
            presetLifting = false;
        } else if (System.currentTimeMillis()-presetStartTime > PRESET_TIMEOUT) {
            presetInMotion = false;
            presetLifting = false;
        }
    }

    public void updateLiftMotors() {
        if (liftTargetPosition < 0) liftTargetPosition = 0;
        if (liftTargetPosition > LIFT_MAX) liftTargetPosition = LIFT_MAX;
        correctLiftError();
        leftLift.setTargetPosition(liftTargetPosition);
        rightLift.setTargetPosition(liftTargetPosition-rightLiftError);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setTargetPosition(liftTargetPosition);
        rightLift.setTargetPosition(liftTargetPosition-rightLiftError);
        leftLift.setVelocity(LIFT_VELOCITY);
        rightLift.setVelocity(LIFT_VELOCITY);
    }

    public void correctLiftError() {
        if ((leftLift.getCurrentPosition()<=liftTargetPosition+LIFT_TOLERANCE&&leftLift.getCurrentPosition()>=liftTargetPosition-LIFT_TOLERANCE)&&leftLift.getVelocity()<LIFT_VELOCITY_TOLERANCE) {
            rightLiftError = leftLift.getCurrentPosition()-rightLift.getCurrentPosition();
        }
    }

    public void asyncTimers() {
        if (resetPixelDrop && (System.currentTimeMillis()-resetPixelDropStartTime > RESET_PIXEL_DROP_WAIT)) {
            resetPixelDrop = false;
            liftTargetPosition = 0;
            updateLiftMotors();
            if (!intakeIn) {
                leftIntake.setPosition(LEFT_INTAKE_DROP_POSITION);
                rightIntake.setPosition(RIGHT_INTAKE_DROP_POSITION);
                leftOuttake.setPosition(LEFT_OUTTAKE_AVOID_POSITION);
                rightOuttake.setPosition(RIGHT_OUTTAKE_AVOID_POSITION);
                resetFoldIn = true;
                resetFoldInStartTime = System.currentTimeMillis();
            } else {
                leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
                rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
                resetInMotion = false;
            }
        }
        if (resetFoldIn && (System.currentTimeMillis()-resetFoldInStartTime > RESET_FOLD_IN_WAIT)) {
            resetFoldIn = true;
            leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
            rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
            resetInMotion = false;
        }

        if (!liftInGrabbingPosition && (leftLift.getCurrentPosition()<=LIFT_GRAB_POSITION+LIFT_TOLERANCE&&leftLift.getCurrentPosition()>=LIFT_GRAB_POSITION-LIFT_TOLERANCE)&&leftLift.getVelocity()<LIFT_VELOCITY_TOLERANCE) {
            liftInGrabbingPosition = true;
            clawGrabbing = true;
            clawGrabbingStartTime = System.currentTimeMillis();
        }
        if (clawGrabbing && (System.currentTimeMillis()-clawGrabbingStartTime > CLAW_GRAB_WAIT)) {
            leftOuttake.setPosition(LEFT_OUTTAKE_GRAB_POSITION);
            rightOuttake.setPosition(RIGHT_OUTTAKE_GRAB_POSITION);
            clawGrabbing = false;
            clawActuallyGrab = true;
        }
        if (clawActuallyGrab && (System.currentTimeMillis()-clawGrabbingStartTime > CLAW_ACTUALLY_GRAB_WAIT + CLAW_GRAB_WAIT)) {
            innerClaw.setPosition(INNER_CLAW_CLOSE_POSITION);
            outerClaw.setPosition(OUTER_CLAW_CLOSE_POSITION);
            clawActuallyGrab = false;
            clawClosing = true;
        }
        if (clawClosing && (System.currentTimeMillis()-clawGrabbingStartTime > CLAW_CLOSE_WAIT + CLAW_ACTUALLY_GRAB_WAIT + CLAW_GRAB_WAIT)) {
            liftTargetPosition = 0;
            updateLiftMotors();
            leftOuttake.setPosition(LEFT_OUTTAKE_OUT_POSITION);
            rightOuttake.setPosition(RIGHT_OUTTAKE_OUT_POSITION);
            clawClosing = false;
            clawLifting = true;
        }
        if (clawLifting && (System.currentTimeMillis()-clawGrabbingStartTime > CLAW_LIFT_WAIT + CLAW_CLOSE_WAIT + CLAW_ACTUALLY_GRAB_WAIT + CLAW_GRAB_WAIT)) {
            clawLifting = false;
            startPreset(presetTargetPosition);
        }

        if (intakeGoingOut && (System.currentTimeMillis()-intakeOutStartTime > INTAKE_FULL_OUT_WAIT)) {
            leftIntake.setPosition(LEFT_INTAKE_OUT_POSITION);
            rightIntake.setPosition(RIGHT_INTAKE_OUT_POSITION);
            intakeGoingOut = false;
        }

        if (intakeGoingOutObstacle && (System.currentTimeMillis()-intakeOutObstacleStartTime > INTAKE_OBSTACLE_OUT_WAIT)) {
            leftIntake.setPosition(LEFT_INTAKE_OUT_PAUSE_POSITION);
            rightIntake.setPosition(RIGHT_INTAKE_OUT_PAUSE_POSITION);
            intakeGoingOutObstacle = false;
            intakeGoingOutObstacleRetract = true;
        }

        if (intakeGoingOutObstacleRetract && (System.currentTimeMillis()-intakeOutObstacleStartTime > INTAKE_OBSTACLE_OUT_RETRACT_WAIT + INTAKE_OBSTACLE_OUT_WAIT)) {
            leftIntake.setPosition(LEFT_INTAKE_OUT_POSITION);
            rightIntake.setPosition(RIGHT_INTAKE_OUT_POSITION);
            leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
            rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
            intakeGoingOutObstacleRetract = false;
        }

        if (intakeGoingIn && (System.currentTimeMillis()-intakeInStartTime)>INTAKE_IN_WAIT) {
            leftIntake.setPosition(LEFT_INTAKE_DROP_POSITION);
            rightIntake.setPosition(RIGHT_INTAKE_DROP_POSITION);
            intakeGoingIn = false;
        }

        if (intakeGoingInObstacle && (System.currentTimeMillis()-intakeInObstacleStartTime>INTAKE_OBSTACLE_IN_WAIT)) {
            leftIntake.setPosition(LEFT_INTAKE_DROP_POSITION);
            rightIntake.setPosition(RIGHT_INTAKE_DROP_POSITION);
            leftOuttake.setPosition(LEFT_OUTTAKE_AVOID_POSITION);
            rightOuttake.setPosition(RIGHT_OUTTAKE_AVOID_POSITION);
            intakeGoingInObstacle = false;
            intakeGoingInObstacleFoldUp = true;
        }

        if (intakeGoingInObstacleFoldUp && (System.currentTimeMillis()-intakeInObstacleStartTime)>OUTTAKE_OBSTACLE_FOLD_IN_WAIT + INTAKE_OBSTACLE_IN_WAIT) {
            leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
            rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
            intakeGoingInObstacleFoldUp = false;
            if (presetQueue) startPreset(presetTargetPosition);
        }
    }

    public void fineAdjustIntakeV4B() {
            if (leftIntake.getPosition() >= LEFT_INTAKE_MIDDLE_POSITION && leftIntake.getPosition() <= LEFT_INTAKE_OUT_POSITION) {
                leftIntake.setPosition(leftIntake.getPosition() + gamepad1.right_stick_y * (deltaTimeNano / 1000000000.0) * INTAKE_CHANGE * INTAKE_DEGREES_TO_SERVO);
                rightIntake.setPosition(1 - leftIntake.getPosition() + RIGHT_INTAKE_OFFSET);
            }
            if (leftIntake.getPosition() < LEFT_INTAKE_MIDDLE_POSITION && -gamepad1.right_stick_y < 0) {
                leftIntake.setPosition(leftIntake.getPosition() + gamepad1.right_stick_y * (deltaTimeNano / 1000000000.0) * INTAKE_CHANGE * INTAKE_DEGREES_TO_SERVO);
                rightIntake.setPosition(1 - leftIntake.getPosition() + RIGHT_INTAKE_OFFSET);
            }
            if (leftIntake.getPosition() > LEFT_INTAKE_OUT_POSITION && -gamepad1.right_stick_y > 0) {
                leftIntake.setPosition(leftIntake.getPosition() + gamepad1.right_stick_y * (deltaTimeNano / 1000000000.0) * INTAKE_CHANGE * INTAKE_DEGREES_TO_SERVO);
                rightIntake.setPosition(1 - leftIntake.getPosition() + RIGHT_INTAKE_OFFSET);
            }
            if (leftIntake.getPosition() > LEFT_INTAKE_OUT_POSITION) {
                leftIntake.setPosition(LEFT_INTAKE_OUT_POSITION);
                rightIntake.setPosition(RIGHT_INTAKE_OUT_POSITION);
            }
            if (leftIntake.getPosition() < LEFT_INTAKE_DROP_POSITION) {
                leftIntake.setPosition(LEFT_INTAKE_DROP_POSITION);
                rightIntake.setPosition(RIGHT_INTAKE_DROP_POSITION);
            }
    }

    public void updateFrameTime() {
        deltaTimeNano = System.nanoTime()- lastFrameTimeNano;
        lastFrameTimeNano = System.nanoTime();
    }
}

/**
 * 8=D
 */