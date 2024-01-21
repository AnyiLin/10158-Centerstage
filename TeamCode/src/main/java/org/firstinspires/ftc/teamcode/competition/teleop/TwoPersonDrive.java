package org.firstinspires.ftc.teamcode.competition.teleop;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_DEGREES_TO_SERVO;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_FINE_ADJUST_LOWER_BOUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_FINE_ADJUST_UPPER_BOUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_IN_TRANSFER_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_PRESET_SPEED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_SERVO_TO_DEGREES;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_VERTICAL_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_AVOID;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSE_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_HIGH_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_LOW_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_MIDDLE_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_TRANSFER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_DEGREES_TO_SERVO;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_DROP_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_OUT_IN_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_PRESET_SPEED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_SERVO_TO_DEGREES;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CLAW_CLOSE_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CLAW_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_GOING_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_GOING_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WRIST_DEGREES_TO_SERVO;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WRIST_FINE_ADJUST_DEGREES_PER_SECOND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WRIST_FINE_ADJUST_LOWER_BOUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WRIST_FINE_ADJUST_UPPER_BOUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WRIST_VERTICAL_OFFSET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.PLANE_LAUNCHER_HOLD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.PLANE_LAUNCHER_LAUNCH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_INTAKE_ARM_OFFSET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_OUTTAKE_ARM_OFFSET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_CLAW_DELAY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_DROPPING;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_GRAB;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_IDLE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_INSTANT_OUT_DELAY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_INTAKE_AVOID;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_INTAKE_AVOID_RUN_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_POSITIONING;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_PRESET_HOLD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_RESET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_RESET_CLAW_DROP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.liftPIDFCoefficients;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.util.NanoTimer;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.SingleRunAction;
import org.firstinspires.ftc.teamcode.util.Timer;

@TeleOp(name = "Two Person Drive", group = "Drive")
public class TwoPersonDrive extends LinearOpMode {

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, liftEncoder;

    public Servo leftIntakeArm, rightIntakeArm, intakeClaw, leftOuttakeArm, rightOuttakeArm, outtakeWrist, outerOuttakeClaw, innerOuttakeClaw, plane;

    public Telemetry telemetryA;

    public VoltageSensor controlHubVoltageSensor;

    public PIDFController liftPIDF;

    public SingleRunAction intakeArmOut, intakeArmIn, outtakeArmIn, outtakeArmOut, outtakeArmWait, liftManualControlReset, startTransfer, highPreset, middlePreset, lowPreset, resetPreset, intakeClawOpen, innerOuttakeClawClose, outerOuttakeClawClose, transferPresetHold, putOuttakeOut, outtakeClawsOpen, transferReset, intakeReset, intakeOut, innerClawToggle, outerClawToggle, intakeClawToggle, intakeArmMoveUpOnePixel, intakeArmMoveDownOnePixel, intakeTransferInTimerReset, setOuttakeWait, resetIntakeInPosition, toggleOuttakeArmPWM, dropOuttakeArm, intakeAvoid;

    public Timer outtakeTimer, transferTimer, pickUpAdjustTimer;

    public NanoTimer frameTimer;

    public boolean autonomous = false, pickUpAdjustIntakeArm, turnOffOuttakeArmPWM;

    public long outtakeMovementTime;

    public double deltaTimeSeconds, outtakeWristDirection, outtakeWristOffset, intakeArmTargetPosition, outtakeArmTargetPosition, intakeArmOutPosition, outtakePreviousStaticPosition, pickUpAdjustDirection;

    public int intakeState, outtakeState, intakeArmTargetState, outtakeArmTargetState, liftTargetPosition, liftPresetTargetPosition, transferState, intakeSpeed, outtakeSpeed;

    public TwoPersonDrive() {
    }

    public TwoPersonDrive(boolean setAuto) {
        autonomous = setAuto;
    }

    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

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

        leftIntakeArm = hardwareMap.get(Servo.class, "leftIntakeArm");
        rightIntakeArm = hardwareMap.get(Servo.class, "rightIntakeArm");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        innerOuttakeClaw = hardwareMap.get(Servo.class, "innerOuttakeClaw");
        outerOuttakeClaw = hardwareMap.get(Servo.class, "outerOuttakeClaw");
        leftOuttakeArm = hardwareMap.get(Servo.class, "leftOuttakeArm");
        rightOuttakeArm = hardwareMap.get(Servo.class, "rightOuttakeArm");
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        plane = hardwareMap.get(Servo.class, "plane");

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        outtakeTimer = new Timer();
        frameTimer = new NanoTimer();
        pickUpAdjustTimer = new Timer();
        transferTimer = new Timer();


        liftPIDF = new PIDFController(liftPIDFCoefficients);

        liftTargetPosition = 0;
        liftPresetTargetPosition = 0;

        liftManualControlReset = new SingleRunAction(() -> setLiftTargetPosition(liftEncoder.getCurrentPosition()));
        intakeArmOut = new SingleRunAction(() -> {
            setIntakeArmInterpolation(intakeArmOutPosition);
        });
        intakeArmIn = new SingleRunAction(() -> {
            setIntakeArmInterpolation(INTAKE_ARM_IN_POSITION);
            intakeArmOutPosition = INTAKE_ARM_OUT_POSITION;
        });
        outtakeArmIn = new SingleRunAction(() -> setOuttakeArmInterpolation(OUTTAKE_ARM_IN_POSITION));
        outtakeArmOut = new SingleRunAction(() -> setOuttakeArmInterpolation(OUTTAKE_ARM_OUT_POSITION));
        outtakeArmWait = new SingleRunAction(() -> {
            if (!MathFunctions.roughlyEquals(leftOuttakeArm.getPosition(), OUTTAKE_ARM_DROP_POSITION)) {
                setOuttakeArmInterpolation(outtakePreviousStaticPosition);
            }
        });
        startTransfer = new SingleRunAction(() -> {
            setLiftTargetPosition(0);
            moveOuttake(OUTTAKE_IN);
            moveIntake(INTAKE_IN);
            innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_OPEN);
            outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
        });
        intakeClawOpen = new SingleRunAction(() -> intakeClaw.setPosition(INTAKE_CLAW_OPEN));
        innerOuttakeClawClose = new SingleRunAction(() -> innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_CLOSED));
        outerOuttakeClawClose = new SingleRunAction(() -> outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_CLOSED));
        transferPresetHold = new SingleRunAction(() -> {
            liftPresetTargetPosition = 0;
            setLiftTargetPosition(liftPresetTargetPosition);
        });
        putOuttakeOut = new SingleRunAction(() -> moveOuttake(OUTTAKE_OUT));
        outtakeClawsOpen = new SingleRunAction(() -> {
            outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
            innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_OPEN);
        });
        transferReset = new SingleRunAction(() -> {
            moveOuttake(OUTTAKE_IN);
            setLiftTargetPosition(0);
        });
        highPreset = new SingleRunAction(() -> {
            liftPresetTargetPosition = LIFT_HIGH_PRESET_POSITION;
            if (transferState == TRANSFER_IDLE) setTransferState(TRANSFER_POSITIONING);
            if (transferState == TRANSFER_PRESET_HOLD) setTransferState(TRANSFER_OUT);
        });
        middlePreset = new SingleRunAction(() -> {
            liftPresetTargetPosition = LIFT_MIDDLE_PRESET_POSITION;
            if (transferState == TRANSFER_IDLE) setTransferState(TRANSFER_POSITIONING);
            if (transferState == TRANSFER_PRESET_HOLD) setTransferState(TRANSFER_OUT);
        });
        lowPreset = new SingleRunAction(() -> {
            liftPresetTargetPosition = LIFT_LOW_PRESET_POSITION;
            if (transferState == TRANSFER_IDLE) setTransferState(TRANSFER_POSITIONING);
            if (transferState == TRANSFER_PRESET_HOLD) setTransferState(TRANSFER_OUT);
        });
        resetPreset = new SingleRunAction(() -> {
            setTransferState(TRANSFER_RESET);
        });
        intakeReset = new SingleRunAction(() -> {
            intakeArmOutPosition = INTAKE_ARM_OUT_POSITION;
            moveIntake(INTAKE_IN);
        });
        intakeOut = new SingleRunAction(() -> {
            moveToCustomIntakeOutPosition(INTAKE_ARM_OUT_POSITION);
        });
        innerClawToggle = new SingleRunAction(() -> {
            if (MathFunctions.roughlyEquals(innerOuttakeClaw.getPosition(), INNER_OUTTAKE_CLAW_CLOSED)) {
                innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_OPEN);
            } else {
                innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_CLOSED);
            }
        });
        outerClawToggle = new SingleRunAction(() -> {
            if (MathFunctions.roughlyEquals(outerOuttakeClaw.getPosition(), OUTER_OUTTAKE_CLAW_CLOSED)) {
                outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
            } else {
                outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_CLOSED);
            }
        });
        intakeClawToggle = new SingleRunAction(() -> {
            if (MathFunctions.roughlyEquals(intakeClaw.getPosition(), INTAKE_CLAW_CLOSED)) {
                intakeClaw.setPosition(INTAKE_CLAW_OPEN);
                if (!autonomous && intakeState == INTAKE_OUT) {
                    pickUpAdjustIntakeArm = true;
                    pickUpAdjustDirection = 1;
                    pickUpAdjustTimer.resetTimer();
                }
            } else {
                intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
                if (!autonomous && intakeState == INTAKE_OUT) {
                    pickUpAdjustIntakeArm = true;
                    pickUpAdjustDirection = -1;
                    pickUpAdjustTimer.resetTimer();
                }
            }
        });
        intakeArmMoveUpOnePixel = new SingleRunAction(() -> {
            if (intakeState == INTAKE_OUT) moveIntakeArmOnePixel(-1);
        });
        intakeArmMoveDownOnePixel = new SingleRunAction(() -> {
            if (intakeState == INTAKE_OUT) moveIntakeArmOnePixel(1);
        });
        intakeTransferInTimerReset = new SingleRunAction(() -> transferTimer.resetTimer());
        setOuttakeWait = new SingleRunAction(() -> setOuttakeState(OUTTAKE_WAIT));
        resetIntakeInPosition = new SingleRunAction(()-> {
            if (intakeState == INTAKE_IN) {
                setIntakeArmInterpolation(INTAKE_ARM_IN_POSITION);
            }
        });
        toggleOuttakeArmPWM = new SingleRunAction(()-> {
            if (turnOffOuttakeArmPWM) {
                turnOffOuttakeArmPWM = false;
                leftOuttakeArm.getController().pwmEnable();
                rightOuttakeArm.getController().pwmEnable();
            } else {
                turnOffOuttakeArmPWM = true;
            }
        });
        dropOuttakeArm = new SingleRunAction(()-> {
            setOuttakeArmInterpolation(OUTTAKE_ARM_DROP_POSITION);
            setOuttakeWristDirection(-5);
        });
        intakeAvoid = new SingleRunAction(()-> setIntakeState(INTAKE_AVOID));

        plane.setPosition(PLANE_LAUNCHER_HOLD);
        leftOuttakeArm.setPosition(OUTTAKE_ARM_IN_POSITION);
        rightOuttakeArm.setPosition(OUTTAKE_ARM_IN_POSITION + RIGHT_OUTTAKE_ARM_OFFSET);
        setIntakeArmPosition(INTAKE_ARM_IN_POSITION);
        outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
        innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_OPEN);
        intakeClaw.setPosition(INTAKE_CLAW_OPEN);
        setOuttakeWristDirection(-15);

        intakeState = INTAKE_IN;
        outtakeState = OUTTAKE_IN;
        outtakeArmTargetState = OUTTAKE_IN;
        intakeArmTargetState = INTAKE_IN;
        outtakeArmTargetPosition = leftOuttakeArm.getPosition();
        setOuttakeArmInterpolation(outtakeArmTargetPosition);
        intakeArmTargetPosition = leftIntakeArm.getPosition();
        setIntakeArmInterpolation(intakeArmTargetPosition);
        outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
        intakeArmOutPosition = INTAKE_ARM_OUT_POSITION;
        transferState = TRANSFER_IDLE;

        setEncoderMotors();

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public void setEncoderMotors() {
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftEncoder = rightLift;
    }

    /**
     * IMPORTANT NOTES:
     * <p>
     * For the Double Shock controllers:
     * triangle - top
     * circle - right
     * cross (X-shaped) - bottom
     * square - left
     * <p>
     * For the older Logitech controllers:
     * Y - top
     * B - right
     * A - bottom
     * X - left
     */
    @Override
    public void runOpMode() {
        work();
    }

    public void work() {
        while (!isStarted() && !isStopRequested()) {
        }

        initialize();

        frameTimer.resetTimer();
        leftIntakeArm.getController().pwmEnable();
        rightIntakeArm.getController().pwmEnable();


        while (opModeIsActive()) {
            driverControlUpdate();

            telemetry();
        }
    }

    public void telemetry() {
        telemetryA.addData("Intake State", intakeState);
        telemetryA.addData("Outtake State", outtakeState);
        telemetryA.addData("Transfer State", transferState);
        telemetryA.addData("Lift Position", liftEncoder.getCurrentPosition());
        telemetryA.addData("Lift Target Position", liftTargetPosition);
        telemetryA.addData("Lift Current", liftEncoder.getCurrent(CurrentUnit.MILLIAMPS));
        telemetryA.addData("Outtake Wrist Offset", outtakeWristOffset);
        telemetryA.addData("Intake Arm Position", leftIntakeArm.getPosition());
        telemetryA.addData("Intake Arm Target Position", intakeArmTargetPosition);
        telemetryA.addData("Intake Arm Target State", intakeArmTargetState);
        telemetryA.addData("Intake Arm PWM Status", leftIntakeArm.getController().getPwmStatus());
        telemetryA.addData("Intake Arm Connection Info", leftIntakeArm.getConnectionInfo());
        telemetryA.update();
    }

    public void driverControlUpdate() {
        updateFrameTime();

        drive();

        buttonControls();

        teleopLiftControlUpdate();

        updateServoMechanisms();

        fineAdjustControls();
    }

    public void autonomousControlUpdate() {
        updateFrameTime();

        updateLift();

        updateServoMechanisms();
    }

    public void drive() {
        double throttle = 0.4 + 0.6 * gamepad1.right_trigger;


        double y = -gamepad1.left_stick_y * throttle; // Remember, this is reversed!

        double x = 0; // this is strafing
        if (gamepad1.left_bumper) {
            x -= 1;
        }
        if (gamepad1.right_bumper) {
            x += 1;
        }
        x *= throttle;

        double rx = 0;
        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            if (MathFunctions.roughlyEquals(0.4, throttle)) {
                rx = gamepad1.left_stick_x * throttle * 0.7;
            } else {
                rx = gamepad1.left_stick_x * throttle;
            }
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        if (controlHubVoltageSensor.getVoltage() < 8.5) {
            denominator *= 2;
        }

        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    public void buttonControls() {
        // plane
        if (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right || gamepad2.dpad_down) {
            plane.setPosition(PLANE_LAUNCHER_LAUNCH);
        }

        // intake presets
        if (gamepad1.circle || gamepad1.b) {
            intakeReset.run();
        } else {
            intakeReset.reset();
        }
        if (gamepad1.a || gamepad1.cross) {
            intakeOut.run();
        } else {
            intakeOut.reset();
        }
        if (gamepad1.right_stick_button) {
            intakeArmMoveUpOnePixel.run();
        } else {
            intakeArmMoveUpOnePixel.reset();
        }
        if (gamepad1.left_stick_button) {
            intakeArmMoveDownOnePixel.run();
        } else {
            intakeArmMoveDownOnePixel.reset();
        }

        // intake claw controls
        if (gamepad1.square || gamepad1.x) {
            intakeClawToggle.run();
        } else {
            intakeClawToggle.reset();
        }

        // outtake claw controls
        if (gamepad2.left_bumper) {
            innerClawToggle.run();
        } else {
            innerClawToggle.reset();
        }
        if (gamepad2.right_bumper) {
            outerClawToggle.run();
        } else {
            outerClawToggle.reset();
        }

        // presets
        if (!turnOffOuttakeArmPWM) {
            if (gamepad2.dpad_up) {
                highPreset.run();
            } else {
                highPreset.reset();
            }
            if (gamepad2.dpad_left) {
                middlePreset.run();
            } else {
                middlePreset.reset();
            }
            if (gamepad2.dpad_down) {
                lowPreset.run();
            } else {
                lowPreset.reset();
            }
            if (gamepad2.dpad_right) {
                resetPreset.run();
            } else {
                resetPreset.reset();
            }
        }

        /*
        // toggle outtake arm on/off for hang
        if (gamepad2.a || gamepad2.cross) {
            toggleOuttakeArmPWM.run();
        } else {
            toggleOuttakeArmPWM.reset();
        }
         */
    }

    public void teleopLiftControlUpdate() {
        if ((outtakeState == OUTTAKE_OUT && transferState == TRANSFER_IDLE) && (Math.abs(gamepad2.left_stick_y) > 0) && ((liftEncoder.getCurrentPosition() < LIFT_MAX_POSITION && liftEncoder.getCurrentPosition() > 0) || (liftEncoder.getCurrentPosition() >= LIFT_MAX_POSITION && -gamepad2.left_stick_y < 0) || (liftEncoder.getCurrentPosition() <= 0 && -gamepad2.left_stick_y > 0))) {
            leftLift.setPower(-gamepad2.left_stick_y);
            rightLift.setPower(-gamepad2.left_stick_y);
            liftManualControlReset.reset();
        } else {
            liftManualControlReset.run();
            updateLift();
        }
    }

    public void updateServoMechanisms() {
        updateTransfer();
        updateIntake();
        updateOuttake();
        updateOuttakeWrist();
        updateIntakeClaw();
    }

    public void updateIntakeClaw() {
        if (pickUpAdjustTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME && pickUpAdjustIntakeArm) {
            pickUpAdjustIntakeArm = false;
            moveIntakeArmOnePixel(pickUpAdjustDirection);
        }
    }

    public void fineAdjustControls() {
        fineAdjustIntakeArm();
        if (gamepad2.right_stick_button) {
            fineAdjustOuttakeWrist();
        } else {
            fineAdjustOuttakeArm();
        }
    }

    public void fineAdjustOuttakeArm() {
        if (outtakeState == OUTTAKE_OUT) {
            if (leftOuttakeArm.getPosition() >= OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND && leftOuttakeArm.getPosition() <= OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND) {
                setOuttakeArmInterpolation(outtakeArmTargetPosition + gamepad2.right_stick_y * deltaTimeSeconds * OUTTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND * OUTTAKE_ARM_DEGREES_TO_SERVO);
            } else {
                if (leftOuttakeArm.getPosition() > OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND) {
                    setOuttakeArmInterpolation(OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND);
                } else if (leftOuttakeArm.getPosition() < OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND) {
                    setOuttakeArmInterpolation(OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND);
                }
            }
        }
    }

    public void fineAdjustOuttakeWrist() {
        if (outtakeState == OUTTAKE_OUT) {
            if (outtakeWristOffset <= OUTTAKE_WRIST_FINE_ADJUST_LOWER_BOUND && outtakeWristOffset >= OUTTAKE_WRIST_FINE_ADJUST_UPPER_BOUND) {
                outtakeWristOffset += gamepad2.right_stick_y * deltaTimeSeconds * OUTTAKE_WRIST_FINE_ADJUST_DEGREES_PER_SECOND;
            } else {
                if (outtakeWristOffset < OUTTAKE_WRIST_FINE_ADJUST_UPPER_BOUND) {
                    outtakeWristOffset = OUTTAKE_WRIST_FINE_ADJUST_UPPER_BOUND;
                } else if (outtakeWristOffset > OUTTAKE_WRIST_FINE_ADJUST_LOWER_BOUND) {
                    outtakeWristOffset = OUTTAKE_WRIST_FINE_ADJUST_LOWER_BOUND;
                }
            }
        }
    }

    public void fineAdjustIntakeArm() {
        if (intakeState == INTAKE_OUT && Math.abs(gamepad1.right_stick_y) > 0) {
            if (leftIntakeArm.getPosition() <= INTAKE_ARM_FINE_ADJUST_LOWER_BOUND && leftIntakeArm.getPosition() >= INTAKE_ARM_FINE_ADJUST_UPPER_BOUND) {
                //setIntakeArmPosition(leftIntakeArm.getPosition() - gamepad1.right_stick_y * deltaTimeSeconds * INTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND * INTAKE_ARM_DEGREES_TO_SERVO);
                //intakeArmOutPosition = leftIntakeArm.getPosition();
                setIntakeArmInterpolation(intakeArmTargetPosition + gamepad1.right_stick_y * deltaTimeSeconds * INTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND * INTAKE_ARM_DEGREES_TO_SERVO);
            } else {
                if (leftIntakeArm.getPosition() < INTAKE_ARM_FINE_ADJUST_UPPER_BOUND) {
                    //setIntakeArmPosition(INTAKE_ARM_FINE_ADJUST_UPPER_BOUND);
                    setIntakeArmInterpolation(INTAKE_ARM_FINE_ADJUST_UPPER_BOUND);
                } else if (leftIntakeArm.getPosition() > INTAKE_ARM_FINE_ADJUST_LOWER_BOUND) {
                    //setIntakeArmPosition(INTAKE_ARM_FINE_ADJUST_LOWER_BOUND);
                    setIntakeArmInterpolation(INTAKE_ARM_FINE_ADJUST_LOWER_BOUND);
                }
            }
        }
    }

    public void moveIntakeArmOnePixel(double direction) {
        direction = MathFunctions.getSign(direction);
        double finalPosition = INTAKE_ARM_VERTICAL_POSITION + INTAKE_ARM_DEGREES_TO_SERVO * (180 - Math.toDegrees(Math.acos(Math.cos(Math.toRadians(180 - ((intakeArmTargetPosition - INTAKE_ARM_VERTICAL_POSITION) * INTAKE_ARM_SERVO_TO_DEGREES))) + (direction * (12.7 / 120.0)))));
        if (finalPosition < INTAKE_ARM_FINE_ADJUST_UPPER_BOUND)
            finalPosition = INTAKE_ARM_FINE_ADJUST_UPPER_BOUND;
        if (finalPosition > INTAKE_ARM_FINE_ADJUST_LOWER_BOUND)
            finalPosition = INTAKE_ARM_FINE_ADJUST_LOWER_BOUND;
        if (!Double.isNaN(finalPosition)) {
            setIntakeArmInterpolation(finalPosition);
        }
    }

    public void setTransferState(int state) {
        switch (state) {
            case TRANSFER_IDLE:
            case TRANSFER_DROPPING:
            case TRANSFER_POSITIONING:
            case TRANSFER_OUT:
            case TRANSFER_RESET:
            case TRANSFER_RESET_CLAW_DROP:
            case TRANSFER_GRAB:
            case TRANSFER_INTAKE_AVOID:
            case TRANSFER_INTAKE_AVOID_RUN_OUT:
            case TRANSFER_PRESET_HOLD:
                transferState = state;
                transferTimer.resetTimer();
                resetTransferActions();
                break;
        }
        updateTransfer();
    }

    public void resetTransferActions() {
        startTransfer.reset();
        intakeClawOpen.reset();
        innerOuttakeClawClose.reset();
        outerOuttakeClawClose.reset();
        transferPresetHold.reset();
        putOuttakeOut.reset();
        outtakeClawsOpen.reset();
        transferReset.reset();
        intakeTransferInTimerReset.reset();
        dropOuttakeArm.reset();
        intakeAvoid.reset();
    }

    public void updateTransfer() {
        if (!(transferState == TRANSFER_IDLE)) {
            switch (transferState) {
                case TRANSFER_POSITIONING:
                    if (MathFunctions.roughlyEquals(intakeClaw.getPosition(), INTAKE_CLAW_OPEN)) {
                        if (MathFunctions.roughlyEquals(outerOuttakeClaw.getPosition(), OUTER_OUTTAKE_CLAW_OPEN) || MathFunctions.roughlyEquals(innerOuttakeClaw.getPosition(), INNER_OUTTAKE_CLAW_OPEN)) {
                            setTransferState(TRANSFER_GRAB);
                        } else {
                            if (intakeState == INTAKE_OUT) {
                                setTransferState(TRANSFER_OUT);
                            } else {
                                setTransferState(TRANSFER_INTAKE_AVOID_RUN_OUT);
                            }
                        }
                        break;
                    }
                    startTransfer.run();
                    if (intakeState == INTAKE_IN && intakeArmAtTargetPosition()) {
                        intakeTransferInTimerReset.run();
                    }
                    if (intakeState == INTAKE_IN  && intakeArmAtTargetPosition() && (transferTimer.getElapsedTime() > INTAKE_ARM_IN_TRANSFER_WAIT) && outtakeState == OUTTAKE_IN && liftEncoder.getCurrentPosition() < LIFT_TRANSFER_UPPER_LIMIT) {
                        setTransferState(TRANSFER_DROPPING);
                    }
                    break;
                case TRANSFER_DROPPING:
                    intakeClawOpen.run();
                    if (transferTimer.getElapsedTime() > TRANSFER_DROP_TIME) {
                        setTransferState(TRANSFER_GRAB);
                    }
                    break;
                case TRANSFER_GRAB:
                    if (transferTimer.getElapsedTime() > 0) {
                        outerOuttakeClawClose.run();
                    }
                    if (transferTimer.getElapsedTime() > TRANSFER_CLAW_DELAY) {
                        innerOuttakeClawClose.run();
                    }
                    if (transferTimer.getElapsedTime() > TRANSFER_CLAW_DELAY + OUTTAKE_CLAW_CLOSE_TIME) {
                        dropOuttakeArm.run();
                    }
                    if (transferTimer.getElapsedTime() > TRANSFER_CLAW_DELAY + OUTTAKE_CLAW_CLOSE_TIME + OUTTAKE_ARM_DROP_TIME) {
                        if (intakeState == INTAKE_OUT) {
                            if (transferTimer.getElapsedTime() > TRANSFER_CLAW_DELAY + OUTTAKE_CLAW_CLOSE_TIME + OUTTAKE_ARM_DROP_TIME) {
                                setTransferState(TRANSFER_PRESET_HOLD);
                            }
                        } else {
                            setTransferState(TRANSFER_INTAKE_AVOID);
                        }
                    }
                    break;
                case TRANSFER_INTAKE_AVOID:
                    intakeAvoid.run();
                    if (intakeArmAtTargetPosition()) {
                        intakeTransferInTimerReset.run();
                    }
                    if (intakeArmAtTargetPosition() && transferTimer.getElapsedTime() > TRANSFER_INSTANT_OUT_DELAY){
                        setTransferState(TRANSFER_PRESET_HOLD);
                    }
                    break;
                case TRANSFER_INTAKE_AVOID_RUN_OUT:
                    intakeAvoid.run();
                    if (intakeArmAtTargetPosition()) {
                        intakeTransferInTimerReset.run();
                    }
                    if (intakeArmAtTargetPosition() && transferTimer.getElapsedTime() > TRANSFER_INSTANT_OUT_DELAY){
                        setTransferState(TRANSFER_OUT);
                    }
                    break;
                case TRANSFER_PRESET_HOLD:
                    transferPresetHold.run();
                    break;
                case TRANSFER_OUT: // todo: FOR AUTO: perhaps make the outtake out position for the transfer kinda close so that we can move the outtake to a farther out position later and place
                    putOuttakeOut.run();
                    if (outtakeState != OUTTAKE_WAIT && intakeState != INTAKE_AVOID) {
                        setLiftTargetPosition(liftPresetTargetPosition);
                        setTransferState(TRANSFER_IDLE);
                    }
                    break;
                case TRANSFER_RESET:
                    if (MathFunctions.roughlyEquals(leftOuttakeArm.getPosition(), OUTTAKE_ARM_DROP_POSITION)) {
                        setOuttakeArmInterpolation(OUTTAKE_ARM_IN_POSITION);
                    }
                    outtakeWristOffset = -7;
                    if (MathFunctions.roughlyEquals(outerOuttakeClaw.getPosition(), OUTER_OUTTAKE_CLAW_CLOSED) || MathFunctions.roughlyEquals(innerOuttakeClaw.getPosition(), INNER_OUTTAKE_CLAW_CLOSED)) {
                        setTransferState(TRANSFER_RESET_CLAW_DROP);
                        break;
                    }
                    transferReset.run();
                    if (outtakeState == OUTTAKE_IN) {
                        setTransferState(TRANSFER_IDLE);
                    }
                    break;
                case TRANSFER_RESET_CLAW_DROP:
                    outtakeClawsOpen.run();
                    if (transferTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                        setTransferState(TRANSFER_RESET);
                    }
                    break;
            }
        }
    }

    public void resetAllActions() {
        resetIntakeActions();
        resetOuttakeActions();
        resetTransferActions();
    }

    public void resetIntakeActions() {
        intakeArmOut.reset();
        intakeArmIn.reset();
        setOuttakeWait.reset();
    }

    public void resetOuttakeActions() {
        outtakeArmIn.reset();
        outtakeArmOut.reset();
        outtakeArmWait.reset();
        resetIntakeInPosition.reset();
    }

    public void moveIntake(int state) {
        switch (state) {
            case INTAKE_IN:
            case INTAKE_OUT:
                intakeArmTargetState = state;
                resetIntakeActions();
                updateIntake();
                break;
        }
    }

    public void moveToCustomIntakeOutPosition(double position) {
        intakeArmOutPosition = position;
        setIntakeArmInterpolation(position);
        moveIntake(INTAKE_OUT);
    }

    public void setIntakeState(int state) {
        switch (state) {
            case INTAKE_IN:
            case INTAKE_OUT:
            case INTAKE_AVOID:
                intakeState = state;
                resetIntakeActions();
                break;
        }
        updateIntake();
    }

    public void updateIntake() {
        switch (intakeArmTargetState) {
            case INTAKE_IN:
                switch (intakeState) {
                    case INTAKE_IN:
                        break;
                    case INTAKE_OUT:
                        if (outtakeState == OUTTAKE_IN && (transferState == TRANSFER_IDLE || transferState == TRANSFER_POSITIONING)) {
                            setIntakeArmInterpolation(INTAKE_ARM_IN_POSITION);
                        } else {
                            setIntakeArmInterpolation(INTAKE_ARM_AVOID_POSITION);
                        }
                        intakeArmOutPosition = INTAKE_ARM_OUT_POSITION;
                        setIntakeState(INTAKE_IN);
                        break;
                    case INTAKE_AVOID:
                        setOuttakeWait.run();
                        setIntakeArmInterpolation(INTAKE_ARM_AVOID_POSITION);
                        if (intakeArmAtTargetPosition()) {
                            if (MathFunctions.roughlyEquals(outtakePreviousStaticPosition, OUTTAKE_ARM_IN_POSITION)) {
                                setOuttakeState(OUTTAKE_IN);
                            } else if (MathFunctions.roughlyEquals(outtakePreviousStaticPosition, OUTTAKE_ARM_OUT_POSITION)) {
                                setOuttakeState(OUTTAKE_OUT);
                            }
                            setIntakeState(INTAKE_IN);
                        }
                        break;
                }
                break;
            case INTAKE_OUT:
                switch (intakeState) {
                    case INTAKE_IN:
                        setIntakeArmInterpolation(intakeArmOutPosition);
                        setIntakeState(INTAKE_OUT);
                        break;
                    case INTAKE_OUT:
                        break;
                    case INTAKE_AVOID:
                        setOuttakeWait.run();
                        setIntakeArmInterpolation(INTAKE_ARM_AVOID_POSITION);
                        if (intakeArmAtTargetPosition()) {
                            if (MathFunctions.roughlyEquals(outtakePreviousStaticPosition, OUTTAKE_ARM_IN_POSITION)) {
                                setOuttakeState(OUTTAKE_IN);
                            } else if (MathFunctions.roughlyEquals(outtakePreviousStaticPosition, OUTTAKE_ARM_OUT_POSITION)) {
                                setOuttakeState(OUTTAKE_OUT);
                            }
                            setIntakeState(INTAKE_IN);
                        }
                        break;
                }
                break;
        }
        updateIntakeArmInterpolation();
    }

    public void moveOuttake(int state) {
        switch (state) {
            case OUTTAKE_IN:
            case OUTTAKE_OUT:
                outtakeArmTargetState = state;
                resetOuttakeActions();
                updateOuttake();
                break;
        }
    }

    public void setOuttakeState(int state) {
        switch (state) {
            case OUTTAKE_IN:
            case OUTTAKE_OUT:
            case OUTTAKE_GOING_IN:
            case OUTTAKE_GOING_OUT:
                outtakeTimer.resetTimer();
                outtakeState = state;
                resetOuttakeActions();
                break;
            case OUTTAKE_WAIT:
                outtakeArmWait.run();
                outtakeTimer.resetTimer();
                outtakeState = state;
                resetOuttakeActions();
                break;
        }
        updateOuttake();
    }

    public void updateOuttake() {
        if (!(outtakeState == OUTTAKE_WAIT)) {
            switch (outtakeArmTargetState) {
                case OUTTAKE_IN:
                    switch (outtakeState) {
                        case OUTTAKE_IN:
                            resetIntakeInPosition.run();
                            if (transferState == TRANSFER_IDLE) {
                                setOuttakeWristDirection(-7);
                                outtakeWristOffset = 0;
                            }
                            outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
                            break;
                        case OUTTAKE_OUT:
                            setOuttakeWristDirection(120);
                            outtakePreviousStaticPosition = OUTTAKE_ARM_OUT_POSITION;
                            if (intakeState == INTAKE_IN && leftIntakeArm.getPosition() > INTAKE_ARM_AVOID_POSITION + 0.0001) {
                                setIntakeState(INTAKE_AVOID);
                            } else if (intakeState == INTAKE_IN && intakeArmTargetPosition > INTAKE_ARM_AVOID_POSITION) {
                                setIntakeArmInterpolation(INTAKE_ARM_AVOID_POSITION);
                            } else if (intakeState == INTAKE_AVOID || (intakeState == INTAKE_OUT && leftIntakeArm.getPosition() < INTAKE_ARM_AVOID_POSITION)) {
                            } else {
                                outtakeMovementTime = OUTTAKE_ARM_OUT_IN_TIME;
                                setOuttakeState(OUTTAKE_GOING_IN);
                            }
                            break;
                        case OUTTAKE_GOING_IN:
                            setOuttakeWristDirection(0);
                            outtakeWristOffset = 0;
                            outtakeArmIn.run();
                            if (outtakeTimer.getElapsedTime() > outtakeMovementTime) {
                                setOuttakeState(OUTTAKE_IN);
                            }
                            break;
                        case OUTTAKE_GOING_OUT:
                            setOuttakeWristDirection(120);
                            outtakeMovementTime = OUTTAKE_ARM_OUT_IN_TIME;
                            setOuttakeState(OUTTAKE_GOING_IN);
                            break;
                    }
                    break;
                case OUTTAKE_OUT:
                    switch (outtakeState) {
                        case OUTTAKE_IN:
                            setOuttakeWristDirection(-7);
                            outtakeWristOffset = 0;
                            outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
                            if (intakeState == INTAKE_IN && leftIntakeArm.getPosition() > INTAKE_ARM_AVOID_POSITION + 0.0001) {
                                setIntakeState(INTAKE_AVOID);
                            } else if (intakeState == INTAKE_IN && intakeArmTargetPosition > INTAKE_ARM_AVOID_POSITION) {
                                setIntakeArmInterpolation(INTAKE_ARM_AVOID_POSITION);
                            } else if (intakeState == INTAKE_AVOID || (intakeState == INTAKE_OUT && leftIntakeArm.getPosition() < INTAKE_ARM_AVOID_POSITION)) { // todo add something to detect for intake going out but not out of the way
                            } else {
                                outtakeMovementTime = OUTTAKE_ARM_OUT_IN_TIME;
                                setOuttakeState(OUTTAKE_GOING_OUT);
                            }
                            break;
                        case OUTTAKE_OUT:
                            setOuttakeWristDirection(120);
                            outtakePreviousStaticPosition = OUTTAKE_ARM_OUT_POSITION;
                            break;
                        case OUTTAKE_GOING_IN:
                            setOuttakeWristDirection(0);
                            outtakeWristOffset = 0;
                            outtakeMovementTime = OUTTAKE_ARM_OUT_IN_TIME;
                            setOuttakeState(OUTTAKE_GOING_OUT);
                            break;
                        case OUTTAKE_GOING_OUT:
                            setOuttakeWristDirection(120);
                            outtakeArmOut.run();
                            if (outtakeTimer.getElapsedTime() > outtakeMovementTime) {
                                setOuttakeState(OUTTAKE_OUT);
                            }
                            break;
                    }
                    break;
            }
        }
        if (turnOffOuttakeArmPWM && (outtakeState == OUTTAKE_IN || outtakeState == OUTTAKE_OUT)) {
            leftOuttakeArm.getController().pwmDisable();
            rightOuttakeArm.getController().pwmDisable();
        }
        updateOuttakeArmInterpolation();
    }

    public void updateLift() {
        liftPIDF.updateError(liftTargetPosition - liftEncoder.getCurrentPosition());
        double liftPower = liftPIDF.runPIDF();
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);
    }

    public void setLiftTargetPosition(int position) {
        liftTargetPosition = position;
        liftPIDF.reset();
        updateLift();
    }

    public void setIntakeArmPosition(double position) {
        if (true){// || !MathFunctions.roughlyEquals(leftIntakeArm.getPosition(), position)) {
            leftIntakeArm.setPosition(position);
            rightIntakeArm.setPosition(1 - position + RIGHT_INTAKE_ARM_OFFSET);
            //rightIntakeArm.setPosition(position);
        }
    }

    public boolean intakeArmAtTargetPosition() {
        return MathFunctions.roughlyEquals(leftIntakeArm.getPosition(), intakeArmTargetPosition);
    }

    public void setIntakeArmInterpolation(double endPosition, int speed) {
        intakeArmTargetPosition = endPosition;
        intakeSpeed = speed;
        updateIntakeArmInterpolation();
    }

    public void setIntakeArmInterpolation(double endPosition) {
        setIntakeArmInterpolation(endPosition, INTAKE_ARM_PRESET_SPEED);
    }

    public void updateIntakeArmInterpolation() {
        double direction = MathFunctions.getSign(intakeArmTargetPosition - leftIntakeArm.getPosition());
        double nextPosition = leftIntakeArm.getPosition() + direction * (double)intakeSpeed * deltaTimeSeconds * INTAKE_ARM_DEGREES_TO_SERVO;
        if ((direction > 0 && nextPosition > intakeArmTargetPosition) || (direction < 0 && nextPosition < intakeArmTargetPosition)) nextPosition = intakeArmTargetPosition;
        setIntakeArmPosition(nextPosition);
    }

    public void setOuttakeArmPosition(double position) {
        if (!MathFunctions.roughlyEquals(leftOuttakeArm.getPosition(), position)) {
            leftOuttakeArm.setPosition(position);
            //rightOuttakeArm.setPosition(1 - position + RIGHT_OUTTAKE_ARM_OFFSET);
            rightOuttakeArm.setPosition(position + RIGHT_OUTTAKE_ARM_OFFSET);
            updateOuttakeWrist();
        }
    }

    public void setOuttakeArmInterpolation(double targetPosition, int speed) {
        outtakeArmTargetPosition = targetPosition;
        outtakeSpeed = speed;
        updateOuttakeArmInterpolation();
    }

    public void setOuttakeArmInterpolation(double targetPosition) {
        setOuttakeArmInterpolation(targetPosition, OUTTAKE_ARM_PRESET_SPEED);
    }

    public void updateOuttakeArmInterpolation() {
        double direction = MathFunctions.getSign(outtakeArmTargetPosition - leftOuttakeArm.getPosition());
        double nextPosition = leftOuttakeArm.getPosition() + direction * (double)outtakeSpeed * deltaTimeSeconds * OUTTAKE_ARM_DEGREES_TO_SERVO;
        if ((direction > 0 && nextPosition > outtakeArmTargetPosition) || (direction < 0 && nextPosition < outtakeArmTargetPosition)) nextPosition = outtakeArmTargetPosition;
        setOuttakeArmPosition(nextPosition);
    }

    public boolean outtakeArmAtTargetPosition() {
        return MathFunctions.roughlyEquals(leftOuttakeArm.getPosition(), outtakeArmTargetPosition);
    }

    /**
     * Sets what direction we want the outtake wrist to point. This will be an absolute direction,
     * so the outtake arm's current angle should not influence the direction the wrist points.
     * 0 is pointing directly upwards and increasing the direction goes out of the robot
     *
     * @param direction the direction the outtake wrist will point. This is an absolute direction
     *                  expressed in degrees
     */
    public void setOuttakeWristDirection(double direction) {
        outtakeWristDirection = direction;
        updateOuttakeWrist();
    }

    /**
     * Updates the wrist's position so it keeps its angle as the outtake arm moves
     */
    public void updateOuttakeWrist() {
        double position = (outtakeWristDirection + outtakeWristOffset) * OUTTAKE_WRIST_DEGREES_TO_SERVO + OUTTAKE_WRIST_VERTICAL_OFFSET - (OUTTAKE_ARM_IN_POSITION - leftOuttakeArm.getPosition()) * OUTTAKE_ARM_SERVO_TO_DEGREES * OUTTAKE_WRIST_DEGREES_TO_SERVO;
        if (!MathFunctions.roughlyEquals(outtakeWrist.getPosition(), position))
            outtakeWrist.setPosition(position);
    }

    public void updateFrameTime() {
        deltaTimeSeconds = frameTimer.getElapsedTimeSeconds();
        frameTimer.resetTimer();
    }
}