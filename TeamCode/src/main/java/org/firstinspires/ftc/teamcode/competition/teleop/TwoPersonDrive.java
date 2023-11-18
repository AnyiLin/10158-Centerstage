package org.firstinspires.ftc.teamcode.competition.teleop;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.BOTTOM_LINE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_CLOSE_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_GRAB_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LIFT_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.DRIVETRAIN_CURRENT_ADJUST_FACTOR;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.DRIVETRAIN_CURRENT_LIMIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.FINE_ADJUST_LIFT_CHANGE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_CLAW_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_CLAW_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_BURST_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CHANGE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_DEGREES_TO_SERVO;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_EDGE_CASE_COLLIDE_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_FULL_OUT_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IN_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OBSTACLE_IN_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OBSTACLE_OUT_RETRACT_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OBSTACLE_OUT_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LEFT_AUTONOMOUS_OUTTAKE_GRAB_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LEFT_INTAKE_DROP_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LEFT_INTAKE_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LEFT_INTAKE_MIDDLE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LEFT_INTAKE_OUT_PAUSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LEFT_INTAKE_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LEFT_OUTTAKE_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LEFT_OUTTAKE_GRAB_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LEFT_OUTTAKE_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LEFT_OUTTAKE_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LEFT_OUTTAKE_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_GO_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_GRAB_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_GRAB_TIMEOUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_GRAB_TOLERANCE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_GRAB_VELOCITY_LIMIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_MAX;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_TOLERANCE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_VELOCITY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.MIDDLE_LINE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_CLAW_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_CLAW_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_AUTONOMOUS_PICK_UP_DEGREES_PER_SECOND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CHANGE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_DEGREES_TO_SERVO;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_FINE_ADJUST_DEAD_ZONE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OBSTACLE_FOLD_IN_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_PICK_UP_DEGREES_PER_SECOND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.PLANE_LAUNCHER_HOLD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.PLANE_LAUNCHER_LAUNCH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.PRESET_TIMEOUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.REGULAR_LIFT_CHANGE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RESET_PIXEL_DROP_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_AUTONOMOUS_OUTTAKE_GRAB_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_INTAKE_DROP_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_INTAKE_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_INTAKE_MIDDLE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_INTAKE_OFFSET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_INTAKE_OUT_PAUSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_INTAKE_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_OUTTAKE_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_OUTTAKE_GRAB_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_OUTTAKE_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_OUTTAKE_OFFSET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_OUTTAKE_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_OUTTAKE_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TOP_LINE_POSITION;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

@TeleOp(name = "Two Person Drive", group = "Drive")
public class TwoPersonDrive extends LinearOpMode {

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, intake;

    public Servo leftIntake, rightIntake, leftOuttake, rightOuttake, outerClaw, innerClaw, planeLauncher;

    public final PIDFCoefficients
            LIFT_UP_VELOCITY_PIDF_COEFFICIENTS = RobotConstants.LIFT_UP_VELOCITY_PIDF_COEFFICIENTS,
            LIFT_UP_POSITION_PIDF_COEFFICIENTS = RobotConstants.LIFT_UP_POSITION_PIDF_COEFFICIENTS,
            LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS = RobotConstants.LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS,
            LIFT_DOWN_POSITION_PIDF_COEFFICIENTS = RobotConstants.LIFT_DOWN_POSITION_PIDF_COEFFICIENTS;

    public boolean outtakeHovering, presetButtonPressed, presetOverride, intakeEdgeCaseCollide, burstToggleButtonPressed, intaking, burstIntake = true, setCustomIntakeOutPosition, useInnerClaw = true, autonomous, adjustingLiftZero, outtakeSlowPickup, outerClawButtonPressed, innerClawButtonPressed, liftGoing, resetFoldIn, resetPixelDrop, liftInGrabbingPosition = true, resetInMotion, clawLifting, clawClosing, outtakeIn, intakeIn, intakeGoingOut, intakeGoingInObstacle, intakeGoingInObstacleFoldUp, intakeGoingIn, intakeGoingOutObstacle, intakeGoingOutObstacleRetract, clawGrabbing, presetInMotion, presetQueue;

    public double customIntakeOutPosition;

    public int leftLiftTargetPosition, rightLiftTargetPosition, presetTargetPosition, leftLiftCurrentAdjust = 0, rightLiftCurrentAdjust = 0, driveTrainCurrentAdjust = 0;

    public long intakeEdgeCaseCollideStartTime, intakingStartTime, liftGrabStartTime, resetPixelDropStartTime, presetStartTime, clawGrabbingStartTime, intakeOutStartTime, intakeInObstacleStartTime, intakeInStartTime, intakeOutObstacleStartTime, lastFrameTimeNano, deltaTimeNano;

    public Telemetry telemetryA;

    public TwoPersonDrive() {
    }

    public TwoPersonDrive(boolean setAuto) {
        autonomous = setAuto;
    }

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

        leftLift.setTargetPositionTolerance(LIFT_TOLERANCE);
        rightLift.setTargetPositionTolerance(LIFT_TOLERANCE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftTargetPosition = 0;
        rightLiftTargetPosition = 0;
        leftLift.setTargetPosition(leftLiftTargetPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setTargetPosition(rightLiftTargetPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");
        leftOuttake = hardwareMap.get(Servo.class, "leftOuttake");
        rightOuttake = hardwareMap.get(Servo.class, "rightOuttake");
        outerClaw = hardwareMap.get(Servo.class, "outerClaw");
        innerClaw = hardwareMap.get(Servo.class, "innerClaw");
        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");

        planeLauncher.setPosition(PLANE_LAUNCHER_HOLD);

        leftIntake.setPosition(LEFT_INTAKE_DROP_POSITION);
        rightIntake.setPosition(RIGHT_INTAKE_DROP_POSITION);
        outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
        innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);
        leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
        rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
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

        burstIntake = false;

        long initStartTime = System.currentTimeMillis();
        while (!isStarted() && !isStopRequested()) {
            if (System.currentTimeMillis()-initStartTime>1500) asyncTimers();
        }

        lastFrameTimeNano = System.nanoTime();
        while (opModeIsActive()) {

            updateFrameTime();

            double throttle = 0.2 + 0.8*gamepad1.right_trigger;

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = 0; // this is strafing

            if (Math.abs(gamepad2.right_stick_x)>1) {
                x = gamepad2.right_stick_x*0.5/throttle;
            } else if (gamepad1.left_bumper) {
                x = -1;
            } else if (gamepad1.right_bumper) {
                x = 1;
            }

            double rx = 0;
            if (Math.abs(gamepad1.left_stick_x)>0.1) rx = gamepad1.left_stick_x;
            if (rx > 1-gamepad1.right_trigger*0.5) {
                rx = 1-gamepad1.right_trigger*0.5;
            } else if (rx < -1+gamepad1.right_trigger*0.5) {
                rx = -1+gamepad1.right_trigger*0.5;
            }

            adjustDriveTrainCurrent();

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1)/throttle;
            double leftFrontPower = (y + x + rx) / denominator - driveTrainCurrentAdjust;
            double leftRearPower = (y - x + rx) / denominator - driveTrainCurrentAdjust;
            double rightFrontPower = (y - x - rx) / denominator - driveTrainCurrentAdjust;
            double rightRearPower = (y + x - rx) / denominator - driveTrainCurrentAdjust;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            if (gamepad1.x || gamepad1.square) {
                updateIntake(gamepad1.left_trigger);
                if (!intaking) {
                    intaking = true;
                    intakingStartTime = System.currentTimeMillis();
                }
            } else if (gamepad1.b || gamepad1.circle) {
                updateIntake(-gamepad1.left_trigger);
                if (!intaking) {
                    intaking = true;
                    intakingStartTime = System.currentTimeMillis();
                }
            } else {
                intake.setVelocity(0);
                intaking = false;
            }

            if (!presetInMotion && !resetInMotion && !outtakeHovering) {
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

            if (gamepad1.right_stick_button) {
                if (!burstToggleButtonPressed) {
                    burstToggleButtonPressed = true;
                    if (burstIntake) {
                        burstIntake = false;
                    } else {
                        burstIntake = true;
                    }
                }
            } else {
                burstToggleButtonPressed = false;
            }

            if (!presetInMotion && !resetInMotion) fineAdjustIntakeV4B();

            if (gamepad1.dpad_right){
                planeLauncher.setPosition(PLANE_LAUNCHER_LAUNCH);
            }


            /*
            any changes to slides is a change in the target position of pid
            left stick controls slides
            d pad is presets
            left trigger is fine adjust slides down
            right trigger is fine adjust slides up
            right bumper toggles claw closest to backdrop
            left bumper toggles claw farther from backdrop
            right stick is fine adjust strafing on the x and fine adjust outtake on the y

            slides presets:
            dpad_up - top line
            dpad_left - middle line
            dpad_down - bottom line
            dpad_right - reset
            */

            if (!presetInMotion && !resetInMotion) {
                if (gamepad2.right_bumper) {
                    if (!outerClawButtonPressed) {
                        outerClawButtonPressed = true;
                        if (outerClaw.getPosition() == OUTER_CLAW_CLOSE_POSITION) {
                            outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
                        } else {
                            outerClaw.setPosition(OUTER_CLAW_CLOSE_POSITION);
                        }
                    }
                } else {
                    outerClawButtonPressed = false;
                }
                if (gamepad2.left_bumper) {
                    if (!innerClawButtonPressed) {
                        innerClawButtonPressed = true;
                        if (innerClaw.getPosition() == INNER_CLAW_CLOSE_POSITION) {
                            innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);
                        } else {
                            innerClaw.setPosition(INNER_CLAW_CLOSE_POSITION);
                        }
                    }
                } else {
                    innerClawButtonPressed = false;
                }
            }

            if (!presetInMotion && !resetInMotion) fineAdjustOuttakeV4B();

            if (!presetInMotion && !resetInMotion) {
                if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
                    adjustingLiftZero = false;
                    leftLiftTargetPosition += (gamepad2.right_trigger - gamepad2.left_trigger) * ((deltaTimeNano / 1000000000.0) * FINE_ADJUST_LIFT_CHANGE);
                } else {
                    if (Math.abs(gamepad2.left_stick_y) > 0.5) {
                        adjustingLiftZero = false;
                        leftLiftTargetPosition += (-gamepad2.left_stick_y) * ((deltaTimeNano / 1000000000.0) * REGULAR_LIFT_CHANGE);
                    }
                }
            }

            if (!resetInMotion) {
                if (gamepad2.dpad_up) {
                    if (!presetButtonPressed && presetInMotion) {
                        presetOverride = true;
                    }
                    presetButtonPressed = true;
                    startPreset(TOP_LINE_POSITION);
                }
                if (gamepad2.dpad_left) {
                    if (!presetButtonPressed && presetInMotion) {
                        presetOverride = true;
                    }
                    presetButtonPressed = true;
                    startPreset(MIDDLE_LINE_POSITION);
                }
                if (gamepad2.dpad_down) {
                    if (!presetButtonPressed && presetInMotion) {
                        presetOverride = true;
                    }
                    presetButtonPressed = true;
                    startPreset(BOTTOM_LINE_POSITION);
                }
                if (!(gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_down)) {
                    presetButtonPressed = false;
                }
                if (gamepad2.dpad_right) {
                    resetPreset();
                }
            }

            asyncTimers();

            //if (presetInMotion) detectPresetEnd();

            if (!adjustingLiftZero) updateLiftMotors();

            if (!presetInMotion && !resetInMotion && gamepad2.right_stick_button) {
                adjustingLiftZero = true;
            }

            runTelemetry();
        }
    }

    public void adjustDriveTrainCurrent() {
        if (leftFront.getCurrent(CurrentUnit.MILLIAMPS) + rightFront.getCurrent(CurrentUnit.MILLIAMPS) + leftRear.getCurrent(CurrentUnit.MILLIAMPS) + rightRear.getCurrent(CurrentUnit.MILLIAMPS) > DRIVETRAIN_CURRENT_LIMIT) {
            driveTrainCurrentAdjust += DRIVETRAIN_CURRENT_ADJUST_FACTOR;
        } else {
            if (driveTrainCurrentAdjust>0) driveTrainCurrentAdjust--;
        }
    }

    public void startPreset(int liftPosition, boolean setUseInnerClaw) {
        if (!presetInMotion) presetStartTime = System.currentTimeMillis();
        presetInMotion = true;
        presetTargetPosition = liftPosition;
        useInnerClaw = setUseInnerClaw;
        adjustingLiftZero = false;
        if (intakeEdgeCaseCollide) {
            return;
        }
        if (outtakeIn) { // if the outtake is in, grab and go
            if (intakeGoingOut || intakeGoingOutObstacle || intakeGoingIn || intakeGoingInObstacle) {
                leftIntake.setPosition(LEFT_INTAKE_DROP_POSITION);
                rightIntake.setPosition(RIGHT_INTAKE_DROP_POSITION);
                leftOuttake.setPosition(LEFT_OUTTAKE_AVOID_POSITION);
                rightOuttake.setPosition(RIGHT_OUTTAKE_AVOID_POSITION);
                intakeIn = true;
                intakeGoingOut = false;
                intakeGoingOutObstacle = false;
                intakeGoingIn = false;
                intakeGoingInObstacle = false;
                intakeEdgeCaseCollide = true;
                intakeEdgeCaseCollideStartTime = System.currentTimeMillis();
            } else {
                if (!intakeIn) {
                    if (!intakeGoingInObstacle && !intakeGoingInObstacleFoldUp) {
                        leftIntake.setPosition(LEFT_INTAKE_MIDDLE_POSITION);
                        rightIntake.setPosition(RIGHT_INTAKE_MIDDLE_POSITION);
                        intakeInObstacleStartTime = System.currentTimeMillis();
                        intakeGoingInObstacle = true;
                    }
                    presetQueue = true;
                    intakeGoingOut = false;
                    intakeGoingOutObstacle = false;
                    intakeGoingOutObstacleRetract = false;
                } else {
                    outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
                    innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);
                    outtakeIn = false;
                    leftLiftTargetPosition = LIFT_GRAB_POSITION;
                    liftInGrabbingPosition = false;
                    liftGrabStartTime = System.currentTimeMillis();
                }
            }
        } else { // if the outtake isnt in, then just go to the position specified
            if (liftInGrabbingPosition && !clawGrabbing && !clawClosing && !clawLifting && !liftGoing) {
                presetInMotion = false;
                leftLiftTargetPosition = presetTargetPosition;
                if (!autonomous) {
                    leftOuttake.setPosition(LEFT_OUTTAKE_AVOID_POSITION);
                    rightOuttake.setPosition(RIGHT_OUTTAKE_AVOID_POSITION);
                    outtakeHovering = false;
                }
            }
        }
    }

    public void startPreset(int liftPosition) {
        startPreset(liftPosition, true);
    }

    public void resetPreset() {
        resetInMotion = true;
        adjustingLiftZero = false;

        outtakeSlowPickup = false;
        presetOverride = false;
        outtakeHovering = false;
        presetQueue = false;
        presetInMotion = false;
        resetFoldIn = false;
        liftInGrabbingPosition = true;
        liftGoing = false;
        clawGrabbing = false;
        clawClosing = false;
        clawLifting = false;

        if (!outtakeIn) {
            innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);
            outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
        }

        outtakeIn = true;

        resetPixelDrop = true;
        resetPixelDropStartTime = System.currentTimeMillis();
    }

    public void detectPresetEnd() {
        if (System.currentTimeMillis()-presetStartTime > PRESET_TIMEOUT) {
            presetInMotion = false;
        }
    }

    public void updateLiftMotors() {
        if (leftLiftTargetPosition < 0 && !presetInMotion && !liftGoing) leftLiftTargetPosition = 0;
        if (leftLiftTargetPosition > LIFT_MAX) leftLiftTargetPosition = LIFT_MAX;
        correctLiftError();
        correctLiftCurrentDraw();
        if (leftLiftTargetPosition >= leftLift.getCurrentPosition()-5) {
            leftLift.setVelocityPIDFCoefficients(LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.p,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.i,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.d,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.f);
            rightLift.setVelocityPIDFCoefficients(LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.p,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.i,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.d,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.f);
            leftLift.setPositionPIDFCoefficients(LIFT_UP_POSITION_PIDF_COEFFICIENTS.p);
            rightLift.setPositionPIDFCoefficients(LIFT_UP_POSITION_PIDF_COEFFICIENTS.p);
        } else {
            leftLift.setVelocityPIDFCoefficients(LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.p,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.i,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.d,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.f);
            rightLift.setVelocityPIDFCoefficients(LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.p,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.i,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.d,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.f);
            leftLift.setPositionPIDFCoefficients(LIFT_DOWN_POSITION_PIDF_COEFFICIENTS.p);
            rightLift.setPositionPIDFCoefficients(LIFT_DOWN_POSITION_PIDF_COEFFICIENTS.p);
        }
        leftLift.setTargetPositionTolerance(LIFT_TOLERANCE);
        rightLift.setTargetPositionTolerance(LIFT_TOLERANCE);
        leftLift.setTargetPosition(leftLiftTargetPosition + leftLiftCurrentAdjust);
        rightLift.setTargetPosition(rightLiftTargetPosition + rightLiftCurrentAdjust);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setTargetPosition(leftLiftTargetPosition + leftLiftCurrentAdjust);
        rightLift.setTargetPosition(rightLiftTargetPosition + rightLiftCurrentAdjust);
        leftLift.setTargetPositionTolerance(LIFT_TOLERANCE);
        rightLift.setTargetPositionTolerance(LIFT_TOLERANCE);
        leftLift.setVelocity(LIFT_VELOCITY);
        rightLift.setVelocity(LIFT_VELOCITY);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void correctLiftError() {
        rightLiftTargetPosition = rightLift.getCurrentPosition()+ leftLiftTargetPosition -leftLift.getCurrentPosition();
    }

    public void correctLiftCurrentDraw() {
        if (leftLift.getCurrent(CurrentUnit.MILLIAMPS)>5000) {
            if (leftLift.getTargetPosition()-leftLift.getCurrentPosition()>0) {
                leftLiftCurrentAdjust--;
            } else {
                leftLiftCurrentAdjust++;
            }
        } else if (leftLiftCurrentAdjust != 0) {
            leftLiftCurrentAdjust -= leftLiftCurrentAdjust/Math.abs(leftLiftCurrentAdjust);
        }
        if (rightLift.getCurrent(CurrentUnit.MILLIAMPS)>5000) {
            if (rightLift.getTargetPosition()-rightLift.getCurrentPosition()>0) {
                rightLiftCurrentAdjust--;
            } else {
                rightLiftCurrentAdjust++;
            }
        } else if (rightLiftCurrentAdjust != 0) {
            rightLiftCurrentAdjust -= rightLiftCurrentAdjust/Math.abs(rightLiftCurrentAdjust);
        }
    }

    public void updateIntake(double factor) {
        if (burstIntake) {
            if (intaking && (((int) ((System.currentTimeMillis() - intakingStartTime) / 100.0) / (int) (INTAKE_BURST_TIME / 100)) % 2 == 0)) {
                intake.setVelocity(INTAKE_VELOCITY * factor);
            } else {
                intake.setVelocity(0);
            }
        } else {
            if (intaking) {
                intake.setVelocity(INTAKE_VELOCITY * factor);
            } else {
                intake.setVelocity(0);
            }
        }
    }

    public void asyncTimers() {
        if (resetPixelDrop && (System.currentTimeMillis()-resetPixelDropStartTime > RESET_PIXEL_DROP_WAIT)) {
            resetPixelDrop = false;
            leftLiftTargetPosition = 0;
            updateLiftMotors();
            if (intakeGoingInObstacle || intakeGoingInObstacleFoldUp || intakeGoingIn || intakeGoingOut || intakeGoingOutObstacle || intakeGoingOutObstacleRetract) {
                leftOuttake.setPosition(LEFT_OUTTAKE_AVOID_POSITION);
                rightOuttake.setPosition(RIGHT_OUTTAKE_AVOID_POSITION);
                resetFoldIn = true;
            } else {
                leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
                rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
            }
            resetInMotion = false;
        }

        if (adjustingLiftZero && ((leftLift.getCurrentPosition()<=0) || leftLift.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)||leftLift.getMode().equals(DcMotor.RunMode.STOP_AND_RESET_ENCODER))) {
            if (leftLift.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftLift.setPower(-0.1);
                rightLift.setPower(-0.1);
                leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            if (leftLift.getCurrent(CurrentUnit.MILLIAMPS)>1100 && leftLift.getZeroPowerBehavior().equals(DcMotor.ZeroPowerBehavior.FLOAT)) {
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftLift.setPower(0.1);
                rightLift.setPower(0.1);
                leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (leftLift.getCurrent(CurrentUnit.MILLIAMPS)>800 && leftLift.getZeroPowerBehavior().equals(DcMotor.ZeroPowerBehavior.BRAKE)) {
                adjustingLiftZero = false;
                leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                if (leftLift.getZeroPowerBehavior().equals(DcMotor.ZeroPowerBehavior.FLOAT)) {
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftLift.setPower(-0.1);
                    rightLift.setPower(-0.1);
                    leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftLift.setPower(0.1);
                    rightLift.setPower(0.1);
                    leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }

        if (intakeEdgeCaseCollide && (System.currentTimeMillis()-intakeEdgeCaseCollideStartTime > INTAKE_EDGE_CASE_COLLIDE_WAIT)) {
            leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
            rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
        }
        if (intakeEdgeCaseCollide && (System.currentTimeMillis()-intakeEdgeCaseCollideStartTime > INTAKE_EDGE_CASE_COLLIDE_WAIT+500)) {
            intakeEdgeCaseCollide = false;
            startPreset(presetTargetPosition, useInnerClaw);
        }
        if (!liftInGrabbingPosition && ((System.currentTimeMillis()-liftGrabStartTime > LIFT_GRAB_TIMEOUT) || (leftLift.getCurrentPosition()<=LIFT_GRAB_POSITION+LIFT_GRAB_TOLERANCE&&leftLift.getCurrentPosition()>=LIFT_GRAB_POSITION-LIFT_GRAB_TOLERANCE && leftLift.getVelocity() < LIFT_GRAB_VELOCITY_LIMIT))) {
            liftInGrabbingPosition = true;
            clawGrabbingStartTime = System.currentTimeMillis();
            if (autonomous) {
                leftOuttake.setPosition(LEFT_AUTONOMOUS_OUTTAKE_GRAB_POSITION);
                rightOuttake.setPosition(RIGHT_AUTONOMOUS_OUTTAKE_GRAB_POSITION);
            } else {
                leftOuttake.setPosition(LEFT_OUTTAKE_GRAB_POSITION);
                rightOuttake.setPosition(RIGHT_OUTTAKE_GRAB_POSITION);

            }
            clawGrabbing = true;
        }
        if (clawGrabbing && (System.currentTimeMillis()-clawGrabbingStartTime > CLAW_GRAB_WAIT)) {
            if (useInnerClaw) innerClaw.setPosition(INNER_CLAW_CLOSE_POSITION);
            outerClaw.setPosition(OUTER_CLAW_CLOSE_POSITION);
            clawGrabbing = false;
            clawClosing = true;
        }
        if (clawClosing && (System.currentTimeMillis()-clawGrabbingStartTime > CLAW_CLOSE_WAIT + CLAW_GRAB_WAIT)) {
            clawClosing = false;
            clawLifting = true;
        }
        if (clawLifting && (System.currentTimeMillis()-clawGrabbingStartTime > CLAW_CLOSE_WAIT + CLAW_GRAB_WAIT)) {
            leftLiftTargetPosition = -100;
            clawLifting = false;
            outtakeSlowPickup = true;
            liftGoing = true;
            updateLiftMotors();
        }
        if (outtakeSlowPickup && ((System.currentTimeMillis()-clawGrabbingStartTime > CLAW_LIFT_WAIT + CLAW_CLOSE_WAIT + CLAW_GRAB_WAIT) && (System.currentTimeMillis()-clawGrabbingStartTime < LIFT_GO_WAIT + CLAW_LIFT_WAIT + CLAW_CLOSE_WAIT + CLAW_GRAB_WAIT))) {
            if (autonomous) {
                if (leftOuttake.getPosition() > LEFT_OUTTAKE_OUT_POSITION) {
                    leftOuttake.setPosition(leftOuttake.getPosition() - (deltaTimeNano / 1000000000.0) * OUTTAKE_AUTONOMOUS_PICK_UP_DEGREES_PER_SECOND  * OUTTAKE_DEGREES_TO_SERVO);
                    rightOuttake.setPosition(1 - leftOuttake.getPosition() + RIGHT_OUTTAKE_OFFSET);
                } else {
                    leftOuttake.setPosition(LEFT_OUTTAKE_OUT_POSITION);
                    rightOuttake.setPosition(RIGHT_OUTTAKE_OUT_POSITION);
                }
            } else {
                if (leftOuttake.getPosition() > LEFT_OUTTAKE_PRESET_POSITION) {
                    leftOuttake.setPosition(leftOuttake.getPosition() - (deltaTimeNano / 1000000000.0) * OUTTAKE_PICK_UP_DEGREES_PER_SECOND * OUTTAKE_DEGREES_TO_SERVO);
                    rightOuttake.setPosition(1 - leftOuttake.getPosition() + RIGHT_OUTTAKE_OFFSET);
                } else {
                    leftOuttake.setPosition(LEFT_OUTTAKE_PRESET_POSITION);
                    rightOuttake.setPosition(RIGHT_OUTTAKE_PRESET_POSITION);
                    clawGrabbingStartTime = System.currentTimeMillis() - (LIFT_GO_WAIT + CLAW_CLOSE_WAIT + CLAW_GRAB_WAIT + CLAW_GRAB_WAIT);
                }
            }
        }
        if (liftGoing && (System.currentTimeMillis()-clawGrabbingStartTime > LIFT_GO_WAIT + CLAW_CLOSE_WAIT + CLAW_GRAB_WAIT + CLAW_GRAB_WAIT)) {
            liftGoing = false;
            outtakeSlowPickup = false;
            if (autonomous) {
                leftOuttake.setPosition(LEFT_OUTTAKE_OUT_POSITION);
                rightOuttake.setPosition(RIGHT_OUTTAKE_OUT_POSITION);
            } else {
                leftOuttake.setPosition(LEFT_OUTTAKE_PRESET_POSITION);
                rightOuttake.setPosition(RIGHT_OUTTAKE_PRESET_POSITION);
            }
            if (autonomous || presetOverride) startPreset(presetTargetPosition);
            if (!presetOverride && !autonomous) outtakeHovering = true;
            presetOverride = false;
            presetInMotion = false;
        }

        if (intakeGoingOut && (System.currentTimeMillis()-intakeOutStartTime > INTAKE_FULL_OUT_WAIT)) {
            if (setCustomIntakeOutPosition) {
                leftIntake.setPosition(customIntakeOutPosition);
                rightIntake.setPosition(1-customIntakeOutPosition+RIGHT_INTAKE_OFFSET);
                setCustomIntakeOutPosition = false;
            } else {
                leftIntake.setPosition(LEFT_INTAKE_OUT_POSITION);
                rightIntake.setPosition(RIGHT_INTAKE_OUT_POSITION);
            }
            intakeGoingOut = false;

            if (resetFoldIn) {
                resetFoldIn = false;
                leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
                rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
            }
        }

        if (intakeGoingOutObstacle && (System.currentTimeMillis()-intakeOutObstacleStartTime > INTAKE_OBSTACLE_OUT_WAIT)) {
            leftIntake.setPosition(LEFT_INTAKE_OUT_PAUSE_POSITION);
            rightIntake.setPosition(RIGHT_INTAKE_OUT_PAUSE_POSITION);
            intakeGoingOutObstacle = false;
            intakeGoingOutObstacleRetract = true;
        }
        if (intakeGoingOutObstacleRetract && (System.currentTimeMillis()-intakeOutObstacleStartTime > INTAKE_OBSTACLE_OUT_RETRACT_WAIT + INTAKE_OBSTACLE_OUT_WAIT)) {
            if (setCustomIntakeOutPosition) {
                leftIntake.setPosition(customIntakeOutPosition);
                rightIntake.setPosition(1-customIntakeOutPosition+RIGHT_INTAKE_OFFSET);
                setCustomIntakeOutPosition = false;
            } else {
                leftIntake.setPosition(LEFT_INTAKE_OUT_POSITION);
                rightIntake.setPosition(RIGHT_INTAKE_OUT_POSITION);
            }

            leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
            rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
            intakeGoingOutObstacleRetract = false;
        }

        if (intakeGoingIn && (System.currentTimeMillis()-intakeInStartTime)>INTAKE_IN_WAIT) {
            leftIntake.setPosition(LEFT_INTAKE_DROP_POSITION);
            rightIntake.setPosition(RIGHT_INTAKE_DROP_POSITION);
            intakeGoingIn = false;

            if (resetFoldIn) {
                resetFoldIn = false;
                leftOuttake.setPosition(LEFT_OUTTAKE_IN_POSITION);
                rightOuttake.setPosition(RIGHT_OUTTAKE_IN_POSITION);
            }
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
            if (presetQueue) {
                presetQueue = false;
                intakeIn = true;
                startPreset(presetTargetPosition);
            }
        }
    }

    public void setCustomIntakeOutPosition(double setPosition) {
        setCustomIntakeOutPosition = true;
        customIntakeOutPosition = setPosition;
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
                leftIntake.setPosition(customIntakeOutPosition);
                rightIntake.setPosition(1-customIntakeOutPosition+RIGHT_INTAKE_OFFSET);
                setCustomIntakeOutPosition = false;
            }
        }
    }

    public void fineAdjustIntakeV4B() {
        if (leftIntake.getPosition() >= LEFT_INTAKE_MIDDLE_POSITION && leftIntake.getPosition() <= LEFT_INTAKE_OUT_POSITION) {
            leftIntake.setPosition(leftIntake.getPosition() + gamepad1.right_stick_y * (deltaTimeNano / 1000000000.0) * INTAKE_CHANGE * INTAKE_DEGREES_TO_SERVO);
            rightIntake.setPosition(1 - leftIntake.getPosition() + RIGHT_INTAKE_OFFSET);
        }
        if ((leftIntake.getPosition() < LEFT_INTAKE_MIDDLE_POSITION && leftIntake.getPosition() > LEFT_INTAKE_IN_POSITION) && -gamepad1.right_stick_y < 0) {
            leftIntake.setPosition(leftIntake.getPosition() + gamepad1.right_stick_y * (deltaTimeNano / 1000000000.0) * INTAKE_CHANGE * INTAKE_DEGREES_TO_SERVO);
            rightIntake.setPosition(1 - leftIntake.getPosition() + RIGHT_INTAKE_OFFSET);
        }
        if (leftIntake.getPosition() > LEFT_INTAKE_OUT_POSITION && gamepad1.right_stick_y > 0) {
            leftIntake.setPosition(leftIntake.getPosition() - gamepad1.right_stick_y * (deltaTimeNano / 1000000000.0) * INTAKE_CHANGE * INTAKE_DEGREES_TO_SERVO);
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

    public void fineAdjustOuttakeV4B() {
        if (Math.abs(gamepad2.right_stick_y) >= OUTTAKE_FINE_ADJUST_DEAD_ZONE) {
            if (leftOuttake.getPosition() < LEFT_OUTTAKE_PRESET_POSITION && leftOuttake.getPosition() > LEFT_OUTTAKE_OUT_POSITION - 90 * OUTTAKE_DEGREES_TO_SERVO) {
                leftOuttake.setPosition(leftOuttake.getPosition() + gamepad2.right_stick_y * (deltaTimeNano / 1000000000.0) * OUTTAKE_CHANGE * OUTTAKE_DEGREES_TO_SERVO);
                rightOuttake.setPosition(1 - leftOuttake.getPosition() + RIGHT_OUTTAKE_OFFSET);
            } else {
                if (leftOuttake.getPosition() <= LEFT_OUTTAKE_OUT_POSITION - 90 * OUTTAKE_DEGREES_TO_SERVO && -gamepad2.right_stick_y < 0) {
                    leftOuttake.setPosition(leftOuttake.getPosition() + gamepad2.right_stick_y * (deltaTimeNano / 1000000000.0) * OUTTAKE_CHANGE * OUTTAKE_DEGREES_TO_SERVO);
                    rightOuttake.setPosition(1 - leftOuttake.getPosition() + RIGHT_OUTTAKE_OFFSET);
                } else {
                    if ((leftOuttake.getPosition() >= LEFT_OUTTAKE_PRESET_POSITION && leftOuttake.getPosition() < LEFT_OUTTAKE_PRESET_POSITION + 20 * OUTTAKE_DEGREES_TO_SERVO) && -gamepad2.right_stick_y > 0) {
                        leftOuttake.setPosition(leftOuttake.getPosition() + gamepad2.right_stick_y * (deltaTimeNano / 1000000000.0) * OUTTAKE_CHANGE * OUTTAKE_DEGREES_TO_SERVO);
                        rightOuttake.setPosition(1 - leftOuttake.getPosition() + RIGHT_OUTTAKE_OFFSET);
                    }
                }
            }
        }
    }

    public void runTelemetry() {
        telemetryA.addData("lift target position", leftLiftTargetPosition);
        telemetryA.addData("left lift current position", leftLift.getCurrentPosition());
        telemetryA.addData("left lift target position", leftLift.getTargetPosition());
        telemetryA.addData("left lift current draw", leftLift.getCurrent(CurrentUnit.MILLIAMPS));
        telemetryA.addData("left lift current adjust", leftLiftCurrentAdjust);
        telemetryA.addData("left lift mode", leftLift.getMode());
        telemetryA.addData("right lift current position", rightLift.getCurrentPosition());
        telemetryA.addData("right lift target position", rightLift.getTargetPosition());
        telemetryA.addData("right lift current draw", rightLift.getCurrent(CurrentUnit.MILLIAMPS));
        telemetryA.addData("right lift current adjust", rightLiftCurrentAdjust);
        telemetryA.addData("right lift mode", rightLift.getMode());
        telemetryA.addData("delta time nano", deltaTimeNano);
        telemetryA.addData("preset in motion", presetInMotion);
        telemetryA.addData("adjustig lift zero", adjustingLiftZero);
        telemetryA.addData("left lift pidf rtp", leftLift.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetryA.addData("left lift pidf velo", leftLift.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetryA.update();
    }

    public void updateFrameTime() {
        deltaTimeNano = System.nanoTime()- lastFrameTimeNano;
        lastFrameTimeNano = System.nanoTime();
    }
}

/**
 * 8=D
 */