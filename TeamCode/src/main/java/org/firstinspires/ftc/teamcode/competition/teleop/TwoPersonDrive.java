package org.firstinspires.ftc.teamcode.competition.teleop;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_AVOID;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_AVOID_POSITION_BUFFER;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_AVOID_RESET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_MANUAL_ADJUST_SPEED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_NOMINAL;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_TRANSFER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_ZERO_RESET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_ZERO_RESET_LIMIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_ZERO_RESET_READY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_ZERO_RESET_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_DEGREES_TO_SERVO;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_FINE_ADJUST_LOWER_BOUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_FINE_ADJUST_UPPER_BOUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_PRESET_SPEED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_SERVO_TO_DEGREES;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_VERTICAL_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_AVOID;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_GOING_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_GOING_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IDLE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_HIGH_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_LOW_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_MIDDLE_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_TRANSFER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_TRANSFER_DELAY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_DEGREES_TO_SERVO;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_OUT_IN_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_SERVO_TO_DEGREES;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CLAW_CLOSE_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CLAW_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_GOING_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_GOING_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IDLE;
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
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_OUTTAKE_ARM_OFFSET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_GRAB;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_IDLE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_POSITIONING;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_PRESET_HOLD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_RESET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_RESET_CLAW_DROP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_TRANSFERRING;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.extensionPIDFCoefficients;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.liftPIDFCoefficients;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
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

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, leftExtension, rightExtension, liftEncoder, extensionEncoder;

    public Servo leftIntakeArm, rightIntakeArm, intakeClaw, leftOuttakeArm, rightOuttakeArm, outtakeWrist, outerOuttakeClaw, innerOuttakeClaw, plane;

    public AnalogInput intakeArmInput;

    public Telemetry telemetryA;

    public VoltageSensor controlHubVoltageSensor;

    public PIDFController liftPIDF, extensionPIDF;

    public SingleRunAction intakeArmOut, intakeArmIn, outtakeArmIn, outtakeArmOut, outtakeArmWait, liftManualControlReset, extensionAvoid, startTransfer, highPreset, middlePreset, lowPreset, resetPreset, intakeClawOpen, innerOuttakeClawClose, outerOuttakeClawClose, transferPresetHold, putOuttakeOut, outtakeClawsOpen, transferReset, intakeReset, intakeOut, innerClawToggle, outerClawToggle, intakeClawToggle, intakeArmMoveUpOnePixel, intakeArmMoveDownOnePixel;

    public Timer outtakeTimer, transferTimer, extensionResetTimer;

    public NanoTimer frameTimer;

    public boolean autonomous = false;

    public long outtakeMovementTime;

    public double deltaTimeSeconds, outtakeWristDirection, outtakeWristOffset, intakeArmTargetPosition, outtakeArmTargetPosition, intakeArmOutPosition, outtakePreviousStaticPosition;

    public int intakeState, outtakeState, intakeArmTargetState, outtakeArmTargetState, liftTargetPosition, liftPresetTargetPosition, extensionTargetPosition, extensionState, transferState, intakeSpeed;

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
        leftExtension = hardwareMap.get(DcMotorEx.class, "leftExtension");
        rightExtension = hardwareMap.get(DcMotorEx.class, "rightExtension");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftExtension.setDirection(DcMotorSimple.Direction.FORWARD);
        rightExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftIntakeArm = hardwareMap.get(Servo.class, "leftIntakeArm");
        rightIntakeArm = hardwareMap.get(Servo.class, "rightIntakeArm");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        innerOuttakeClaw = hardwareMap.get(Servo.class, "innerOuttakeClaw");
        outerOuttakeClaw = hardwareMap.get(Servo.class, "outerOuttakeClaw");
        leftOuttakeArm = hardwareMap.get(Servo.class, "leftOuttakeArm");
        rightOuttakeArm = hardwareMap.get(Servo.class, "rightOuttakeArm");
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        plane = hardwareMap.get(Servo.class, "plane");

        intakeArmInput = hardwareMap.get(AnalogInput.class, "intakeArmInput");


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        outtakeTimer = new Timer();
        transferTimer = new Timer();
        extensionResetTimer = new Timer();
        frameTimer = new NanoTimer();


        liftPIDF = new PIDFController(liftPIDFCoefficients);
        extensionPIDF = new PIDFController(extensionPIDFCoefficients);

        liftTargetPosition = 0;
        liftPresetTargetPosition = 0;
        extensionTargetPosition = 0;

        liftManualControlReset = new SingleRunAction(() -> setLiftTargetPosition(liftEncoder.getCurrentPosition()));
        extensionAvoid = new SingleRunAction(() -> {
            setExtensionTargetPosition(EXTENSION_AVOID_POSITION + EXTENSION_AVOID_POSITION_BUFFER);
        });
        intakeArmOut = new SingleRunAction(() -> {
            setIntakeArmInterpolation(intakeArmOutPosition);
            intakeArmOutPosition = INTAKE_ARM_OUT_POSITION;
        });
        intakeArmIn = new SingleRunAction(() -> {
            setIntakeArmInterpolation(INTAKE_ARM_IN_POSITION);
        });
        outtakeArmIn = new SingleRunAction(() -> setOuttakeArmPosition(OUTTAKE_ARM_IN_POSITION));
        outtakeArmOut = new SingleRunAction(() -> setOuttakeArmPosition(OUTTAKE_ARM_OUT_POSITION));
        outtakeArmWait = new SingleRunAction(() -> setOuttakeArmPosition(outtakePreviousStaticPosition));
        startTransfer = new SingleRunAction(() -> {
            setExtensionTargetPosition(0);
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
            moveOuttake(OUTTAKE_IN);
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
        intakeReset = new SingleRunAction(()-> {
            setExtensionTargetPosition(0);
            intakeArmOutPosition = INTAKE_ARM_OUT_POSITION;
            moveIntake(INTAKE_IN);
        });
        intakeOut = new SingleRunAction(()-> {
            intakeArmOutPosition = INTAKE_ARM_OUT_POSITION;
            intakeArmTargetPosition = INTAKE_ARM_OUT_POSITION;
            setIntakeArmInterpolation(INTAKE_ARM_OUT_POSITION);
            moveIntake(INTAKE_OUT);
        });
        innerClawToggle = new SingleRunAction(()-> {
            if (MathFunctions.roughlyEquals(innerOuttakeClaw.getPosition(), INNER_OUTTAKE_CLAW_CLOSED)) {
                innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_OPEN);
            } else {
                innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_CLOSED);
            }
        });
        outerClawToggle = new SingleRunAction(()-> {
            if (MathFunctions.roughlyEquals(outerOuttakeClaw.getPosition(), OUTER_OUTTAKE_CLAW_CLOSED)) {
                outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
            } else {
                outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_CLOSED);
            }
        });
        intakeClawToggle = new SingleRunAction(()-> {
            if (MathFunctions.roughlyEquals(intakeClaw.getPosition(), INTAKE_CLAW_CLOSED)) {
                intakeClaw.setPosition(INTAKE_CLAW_OPEN);
            } else {
                intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
            }
        });
        intakeArmMoveUpOnePixel = new SingleRunAction(()-> {
            if (intakeState == INTAKE_OUT) moveIntakeArmOnePixel(1);
        });
        intakeArmMoveDownOnePixel = new SingleRunAction(()-> {
            if (intakeState == INTAKE_OUT) moveIntakeArmOnePixel(-1);
        });

        plane.setPosition(PLANE_LAUNCHER_HOLD);
        setOuttakeArmPosition(OUTTAKE_ARM_IN_POSITION);
        setIntakeArmPosition(INTAKE_ARM_IN_POSITION);
        outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
        innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_OPEN);
        intakeClaw.setPosition(INTAKE_CLAW_OPEN);
        setOuttakeWristDirection(0);

        intakeState = INTAKE_IN;
        outtakeState = OUTTAKE_IN;
        outtakeArmTargetState = OUTTAKE_IN;
        intakeArmTargetState = INTAKE_IN;
        outtakeArmTargetPosition = leftOuttakeArm.getPosition();
        intakeArmTargetPosition = leftIntakeArm.getPosition();
        outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
        intakeArmOutPosition = INTAKE_ARM_OUT_POSITION;
        extensionState = EXTENSION_NOMINAL;
        transferState = TRANSFER_IDLE;

        setEncoderMotors();

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public void setEncoderMotors() {
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftEncoder = rightLift;
        extensionEncoder = rightExtension;
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
        telemetryA.addData("Extension State", extensionState);
        telemetryA.addData("Transfer State", transferState);
        telemetryA.addData("Lift Position", liftEncoder.getCurrentPosition());
        telemetryA.addData("Lift Target Position", liftTargetPosition);
        telemetryA.addData("Lift Current", liftEncoder.getCurrent(CurrentUnit.MILLIAMPS));
        telemetryA.addData("Extension Position", extensionEncoder.getCurrentPosition());
        telemetryA.addData("Extension Target Position", extensionTargetPosition);
        telemetryA.addData("Extension Current", extensionEncoder.getCurrent(CurrentUnit.MILLIAMPS));
        telemetryA.addData("Outtake Wrist Offset", outtakeWristOffset);
        telemetryA.addData("Intake Arm Position", leftIntakeArm.getPosition());
        telemetryA.addData("Intake Arm PWM Status", leftIntakeArm.getController().getPwmStatus());
        telemetryA.addData("Intake Arm Connection Info", leftIntakeArm.getConnectionInfo());
        telemetryA.update();
    }

    public void driverControlUpdate() {
        updateFrameTime();

        drive();

        buttonControls();

        teleopLiftControlUpdate();

        updateExtension();

        updateServoMechanisms();

        fineAdjustControls();
    }

    public void autonomousControlUpdate() {
        updateFrameTime();

        updateLift();

        updateExtension();

        updateServoMechanisms();
    }

    public void drive() {
        double throttle = 0.4 + 0.6*gamepad1.right_trigger;


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
        if (Math.abs(gamepad1.left_stick_x)>0.1) rx = gamepad1.left_stick_x * throttle;

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
        if (gamepad1.left_stick_button) {
            intakeArmMoveUpOnePixel.run();
        } else {
            intakeArmMoveUpOnePixel.reset();
        }
        if (gamepad1.right_stick_button) {
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

    public void extensionMovementUpdate() {
        if (extensionManualControlsActive()) {
            if (!(gamepad1.left_trigger > 0 || (gamepad1.triangle || gamepad1.y))) {
                // yoyi controls
                extensionTargetPosition += (gamepad2.right_trigger - gamepad2.left_trigger) * deltaTimeSeconds * EXTENSION_MANUAL_ADJUST_SPEED;
                extensionTargetPosition = (int) MathFunctions.clamp(extensionTargetPosition, 0, EXTENSION_MAX_POSITION);
                updateExtensionPIDF();
            } else {
                // jack controls
                if (gamepad1.triangle || gamepad1.y) {
                    extensionTargetPosition -= deltaTimeSeconds * EXTENSION_MANUAL_ADJUST_SPEED;
                    extensionTargetPosition = (int) MathFunctions.clamp(extensionTargetPosition, 0, EXTENSION_MAX_POSITION);
                    updateExtensionPIDF();
                } else {
                    if (extensionEncoder.getCurrentPosition() < EXTENSION_MAX_POSITION) {
                        leftExtension.setPower(gamepad1.left_trigger);
                        rightExtension.setPower(gamepad1.left_trigger);
                        extensionTargetPosition = extensionEncoder.getCurrentPosition();
                    }
                }
            }
        }
        else {
            updateExtensionPIDF();
        }
    }

    public boolean extensionManualControlsActive() {
        return (!autonomous) && (gamepad1.left_trigger > 0 || (gamepad1.triangle || gamepad1.y) || (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0));
    }

    public void updateServoMechanisms() {
        updateTransfer();
        updateIntake();
        updateOuttake();
        updateOuttakeWrist();
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
                setOuttakeArmPosition(leftOuttakeArm.getPosition() + gamepad2.right_stick_y * deltaTimeSeconds * OUTTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND * OUTTAKE_ARM_DEGREES_TO_SERVO);
            } else {
                if (leftOuttakeArm.getPosition() > OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND) {
                    setOuttakeArmPosition(OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND);
                } else if (leftOuttakeArm.getPosition() < OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND) {
                    setOuttakeArmPosition(OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND);
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
        if (intakeState == INTAKE_OUT) {
            if (leftIntakeArm.getPosition() >= INTAKE_ARM_FINE_ADJUST_LOWER_BOUND && leftIntakeArm.getPosition() <= INTAKE_ARM_FINE_ADJUST_UPPER_BOUND) {
                setIntakeArmPosition(leftIntakeArm.getPosition() - gamepad1.right_stick_y * deltaTimeSeconds * INTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND * INTAKE_ARM_DEGREES_TO_SERVO);
                //intakeArmOutPosition = leftIntakeArm.getPosition();
                intakeArmTargetPosition = leftIntakeArm.getPosition();
            } else {
                if (leftIntakeArm.getPosition() > INTAKE_ARM_FINE_ADJUST_UPPER_BOUND) {
                    setIntakeArmPosition(INTAKE_ARM_FINE_ADJUST_UPPER_BOUND);
                    intakeArmTargetPosition = leftIntakeArm.getPosition();
                } else if (leftIntakeArm.getPosition() < INTAKE_ARM_FINE_ADJUST_LOWER_BOUND) {
                    setIntakeArmPosition(INTAKE_ARM_FINE_ADJUST_LOWER_BOUND);
                    intakeArmTargetPosition = leftIntakeArm.getPosition();
                }
            }
        }
    }

    public void moveIntakeArmOnePixel(double direction) {
        direction = MathFunctions.getSign(direction);
        double finalPosition = INTAKE_ARM_VERTICAL_POSITION - INTAKE_ARM_DEGREES_TO_SERVO * (180 - Math.toDegrees(Math.acos(Math.cos(Math.toRadians(180 - ((INTAKE_ARM_VERTICAL_POSITION - leftIntakeArm.getPosition()) * INTAKE_ARM_SERVO_TO_DEGREES))) + (direction * (12.7/294.0)))));
        if (finalPosition > INTAKE_ARM_FINE_ADJUST_UPPER_BOUND) finalPosition = INTAKE_ARM_FINE_ADJUST_UPPER_BOUND;
        if (finalPosition < INTAKE_ARM_FINE_ADJUST_LOWER_BOUND) finalPosition = INTAKE_ARM_FINE_ADJUST_LOWER_BOUND;
        setIntakeArmPosition(finalPosition);
        intakeArmTargetPosition = finalPosition;
    }

    public void setTransferState(int state) {
        switch (state) {
            case TRANSFER_IDLE:
            case TRANSFER_TRANSFERRING:
            case TRANSFER_POSITIONING:
            case TRANSFER_OUT:
            case TRANSFER_RESET:
            case TRANSFER_RESET_CLAW_DROP:
            case TRANSFER_GRAB:
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
    }

    public void updateTransfer() {
        if (!(transferState == TRANSFER_IDLE)) {
            switch (transferState) {
                case TRANSFER_POSITIONING:
                    if (MathFunctions.roughlyEquals(intakeClaw.getPosition(), INTAKE_CLAW_OPEN)) {
                        if (MathFunctions.roughlyEquals(outerOuttakeClaw.getPosition(), OUTER_OUTTAKE_CLAW_OPEN) || MathFunctions.roughlyEquals(innerOuttakeClaw.getPosition(), INNER_OUTTAKE_CLAW_OPEN)) {
                            setTransferState(TRANSFER_GRAB);
                        } else {
                            setTransferState(TRANSFER_OUT);
                        }
                        break;
                    }
                    startTransfer.run();
                    if (intakeState == INTAKE_IN && outtakeState == OUTTAKE_IN && (extensionState == EXTENSION_NOMINAL || extensionState == EXTENSION_ZERO_RESET || extensionState == EXTENSION_ZERO_RESET_READY) && extensionEncoder.getCurrentPosition() < EXTENSION_TRANSFER_UPPER_LIMIT && liftEncoder.getCurrentPosition() < LIFT_TRANSFER_UPPER_LIMIT) {
                        setTransferState(TRANSFER_TRANSFERRING);
                    }
                    break;
                case TRANSFER_TRANSFERRING:
                    intakeClawOpen.run();
                    if (transferTimer.getElapsedTime() > TRANSFER_DROP_TIME) {
                        innerOuttakeClawClose.run();
                    }
                    if (transferTimer.getElapsedTime() > TRANSFER_DROP_TIME + OUTER_OUTTAKE_CLAW_TRANSFER_DELAY) {
                        outerOuttakeClawClose.run();
                    }
                    if (transferTimer.getElapsedTime() > TRANSFER_DROP_TIME + OUTER_OUTTAKE_CLAW_TRANSFER_DELAY + OUTTAKE_CLAW_CLOSE_TIME) {
                        if (autonomous) {
                            setTransferState(TRANSFER_OUT);
                        } else {
                            setTransferState(TRANSFER_PRESET_HOLD);
                        }
                    }
                    break;
                case TRANSFER_GRAB:
                    if (transferTimer.getElapsedTime() > 0) {
                        innerOuttakeClawClose.run();
                    }
                    if (transferTimer.getElapsedTime() > OUTER_OUTTAKE_CLAW_TRANSFER_DELAY) {
                        outerOuttakeClawClose.run();
                    }
                    if (transferTimer.getElapsedTime() > OUTER_OUTTAKE_CLAW_TRANSFER_DELAY + OUTTAKE_CLAW_CLOSE_TIME) {
                        if (autonomous) {
                            setTransferState(TRANSFER_OUT);
                        } else {
                            setTransferState(TRANSFER_PRESET_HOLD);
                        }
                    }
                    break;
                case TRANSFER_PRESET_HOLD:
                    transferPresetHold.run();
                    break;
                case TRANSFER_OUT:
                    putOuttakeOut.run();
                    if (outtakeState != OUTTAKE_WAIT && intakeState != INTAKE_AVOID) {
                        setLiftTargetPosition(liftPresetTargetPosition);
                        setTransferState(TRANSFER_IDLE);
                    }
                    break;
                case TRANSFER_RESET:
                    outtakeWristOffset = 0;
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
    }

    public void resetOuttakeActions() {
        outtakeArmIn.reset();
        outtakeArmOut.reset();
        outtakeArmWait.reset();
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
        intakeArmTargetPosition = position;
        moveIntake(INTAKE_OUT);
    }

    public void setIntakeState(int state) {
        switch (state) {
            case INTAKE_IN:
            case INTAKE_OUT:
            case INTAKE_AVOID:
            case INTAKE_GOING_IN:
            case INTAKE_GOING_OUT:
            case INTAKE_IDLE:
                intakeState = state;
                resetIntakeActions();
                break;
        }
        updateIntake();
    }

    public void updateIntake() {
        if (!(intakeState == INTAKE_IDLE)) {
            switch (intakeArmTargetState) {
                case INTAKE_IN:
                    switch (intakeState) {
                        case INTAKE_IN:
                            break;
                        case INTAKE_OUT:
                            if (outtakeState == OUTTAKE_GOING_IN || outtakeState == OUTTAKE_OUT) {
                                setIntakeState(INTAKE_AVOID);
                            } else {
                                setIntakeState(INTAKE_GOING_IN);
                            }
                            break;
                        case INTAKE_GOING_IN:
                            intakeArmIn.run();
                            updateIntakeArmInterpolation();
                            if (MathFunctions.roughlyEquals(leftIntakeArm.getPosition(), intakeArmTargetPosition)) {
                                setIntakeState(INTAKE_IN);
                            }
                            break;
                        case INTAKE_GOING_OUT:
                            setIntakeState(INTAKE_GOING_IN);
                            break;
                        case INTAKE_AVOID:
                            setOuttakeState(OUTTAKE_WAIT);
                            if (extensionTargetPosition < EXTENSION_AVOID_POSITION) {
                                setExtensionState(EXTENSION_AVOID);
                            }
                            if (extensionEncoder.getCurrentPosition() > EXTENSION_AVOID_POSITION) {
                                if (MathFunctions.roughlyEquals(outtakePreviousStaticPosition, OUTTAKE_ARM_IN_POSITION)) {
                                    setOuttakeState(OUTTAKE_IN);
                                } else if (MathFunctions.roughlyEquals(outtakePreviousStaticPosition, OUTTAKE_ARM_OUT_POSITION)) {
                                    setOuttakeState(OUTTAKE_OUT);
                                }
                                setIntakeState(INTAKE_GOING_IN);
                            }
                            break;
                    }
                    break;
                case INTAKE_OUT:
                    switch (intakeState) {
                        case INTAKE_IN:
                            if (outtakeState == OUTTAKE_GOING_IN || outtakeState == OUTTAKE_OUT) {
                                // set the outtake to move back to its previous static state
                                // and push the intake to avoiding
                                // when the intake is out of the way, put the outtake in and then fold up
                                setIntakeState(INTAKE_AVOID);
                            } else {
                                setIntakeState(INTAKE_GOING_OUT);
                            }
                            break;
                        case INTAKE_OUT:
                            updateIntakeArmInterpolation();
                            /*
                            if (!MathFunctions.roughlyEquals(leftIntakeArm.getPosition(), intakeArmOutPosition)) {
                                setIntakeState(INTAKE_OUT_ADJUSTING);
                            }
                             */
                            break;
                        case INTAKE_GOING_IN:
                            setIntakeState(INTAKE_GOING_OUT);
                            break;
                        case INTAKE_GOING_OUT:
                            intakeArmOut.run();
                            updateIntakeArmInterpolation();
                            if (MathFunctions.roughlyEquals(leftIntakeArm.getPosition(), intakeArmTargetPosition)) {
                                setIntakeState(INTAKE_OUT);
                            }
                            break;
                        case INTAKE_AVOID:
                            setOuttakeState(OUTTAKE_WAIT);
                            if (extensionTargetPosition < EXTENSION_AVOID_POSITION)
                                setExtensionState(EXTENSION_AVOID);
                            if (extensionEncoder.getCurrentPosition() > EXTENSION_AVOID_POSITION) {
                                if (MathFunctions.roughlyEquals(outtakePreviousStaticPosition, OUTTAKE_ARM_IN_POSITION)) {
                                    setOuttakeState(OUTTAKE_IN);
                                } else if (MathFunctions.roughlyEquals(outtakePreviousStaticPosition, OUTTAKE_ARM_OUT_POSITION)) {
                                    setOuttakeState(OUTTAKE_OUT);
                                }
                                setIntakeState(INTAKE_GOING_OUT);
                            }
                            break;
                    }
                    break;
            }
        }
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
            case OUTTAKE_IDLE:
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
        if (!(outtakeState == OUTTAKE_WAIT || outtakeState == OUTTAKE_IDLE)) {
            switch (outtakeArmTargetState) {
                case OUTTAKE_IN:
                    switch (outtakeState) {
                        case OUTTAKE_IN:
                            setOuttakeWristDirection(0);
                            outtakeWristOffset = 0;
                            outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
                            if (extensionState == EXTENSION_AVOID)
                                setExtensionState(EXTENSION_AVOID_RESET);
                            break;
                        case OUTTAKE_OUT:
                            setOuttakeWristDirection(120);
                            outtakePreviousStaticPosition = OUTTAKE_ARM_OUT_POSITION;
                            if ((intakeState == INTAKE_IN || intakeState == INTAKE_GOING_IN || intakeState == INTAKE_GOING_OUT) && (extensionTargetPosition < EXTENSION_AVOID_POSITION)) {
                                setIntakeState(INTAKE_AVOID);
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
                            setOuttakeWristDirection(0);
                            outtakeWristOffset = 0;
                            outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
                            if ((intakeState == INTAKE_IN || intakeState == INTAKE_GOING_IN || intakeState == INTAKE_GOING_OUT) && (extensionTargetPosition < EXTENSION_AVOID_POSITION)) {
                                setIntakeState(INTAKE_AVOID);
                            } else {
                                outtakeMovementTime = OUTTAKE_ARM_OUT_IN_TIME;
                                setOuttakeState(OUTTAKE_GOING_OUT);
                            }
                            break;
                        case OUTTAKE_OUT:
                            setOuttakeWristDirection(120);
                            outtakePreviousStaticPosition = OUTTAKE_ARM_OUT_POSITION;
                            if (extensionState == EXTENSION_AVOID)
                                setExtensionState(EXTENSION_AVOID_RESET);
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
    }

    public void updateAllSlides() {
        updateLift();
        updateExtension();
    }

    public void updateLift() {
        liftPIDF.updateError(liftTargetPosition - liftEncoder.getCurrentPosition());
        double liftPower = liftPIDF.runPIDF();
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);
    }

    public void updateExtension() {
        switch (extensionState) {
            case EXTENSION_NOMINAL:
                extensionMovementUpdate();
                break;
            case EXTENSION_ZERO_RESET_READY:
                if (extensionEncoder.getCurrentPosition() < EXTENSION_ZERO_RESET_LIMIT) {
                    setExtensionState(EXTENSION_ZERO_RESET);
                    break;
                }
                extensionMovementUpdate();
                break;
            case EXTENSION_ZERO_RESET:
                if (transferTimer.getElapsedTime() > EXTENSION_ZERO_RESET_TIME) {
                    leftExtension.setPower(0.01);
                    rightExtension.setPower(0.01);
                }
                if (transferTimer.getElapsedTime() > EXTENSION_ZERO_RESET_TIME+200) {
                    extensionEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extensionEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extensionTargetPosition = 0;
                    setExtensionState(EXTENSION_NOMINAL);
                    break;
                }
                if (extensionManualControlsActive()) {
                    setExtensionState(EXTENSION_ZERO_RESET_READY);
                    break;
                }
                break;
            case EXTENSION_AVOID:
                if (extensionTargetPosition < EXTENSION_AVOID_POSITION) {
                    extensionAvoid.run();
                    break;
                } else {
                    extensionAvoid.reset();
                }
                if (extensionEncoder.getCurrentPosition() >= EXTENSION_AVOID_POSITION) {
                    extensionMovementUpdate();
                    break;
                }
                break;
        }
    }

    public void updateExtensionPIDF() {
        double error = extensionTargetPosition - extensionEncoder.getCurrentPosition();
        if (Math.abs(error) > 1.0/extensionPIDF.P()) {
            leftExtension.setPower(MathFunctions.getSign(error));
            rightExtension.setPower(MathFunctions.getSign(error));
        } else {
            extensionPIDF.updateError(error);
            double extensionPower = extensionPIDF.runPIDF();
            if (extensionEncoder.getCurrent(CurrentUnit.MILLIAMPS) > 3000) extensionPower /= 2;
            leftExtension.setPower(extensionPower);
            rightExtension.setPower(extensionPower);
        }
    }

    public void setLiftTargetPosition(int position) {
        liftTargetPosition = position;
        liftPIDF.reset();
        updateLift();
    }

    public void setExtensionTargetPosition(int position) {
        extensionTargetPosition = position;
        extensionPIDF.reset();
        updateExtensionPIDF();
    }

    public void setExtensionState(int state) {
        switch (state) {
            case EXTENSION_ZERO_RESET_READY:
            case EXTENSION_NOMINAL:
                extensionState = state;
                break;
            case EXTENSION_ZERO_RESET:
                extensionState = state;
                extensionResetTimer.resetTimer();
                leftExtension.setPower(-1);
                rightExtension.setPower(-1);
                break;
            case EXTENSION_AVOID:
                extensionState = state;
                setExtensionTargetPosition(EXTENSION_AVOID_POSITION + EXTENSION_AVOID_POSITION_BUFFER);
                break;
            case EXTENSION_AVOID_RESET:
                extensionState = EXTENSION_ZERO_RESET_READY;
                if (extensionTargetPosition == EXTENSION_AVOID_POSITION + EXTENSION_AVOID_POSITION_BUFFER) setExtensionTargetPosition(0);
                break;
        }
        resetExtensionActions();
        updateExtension();
    }

    public void resetExtensionActions() {
        extensionAvoid.reset();
    }

    public void setIntakeArmPosition(double position) {
        if (!MathFunctions.roughlyEquals(leftIntakeArm.getPosition(), position)) {
            leftIntakeArm.setPosition(position);
            //rightIntakeArm.setPosition(1 - position + RIGHT_INTAKE_ARM_OFFSET);
            rightIntakeArm.setPosition(position);
        }
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
        double nextPosition = leftIntakeArm.getPosition() + direction * intakeSpeed * deltaTimeSeconds * INTAKE_ARM_DEGREES_TO_SERVO;
        if ((direction > 0 && nextPosition > intakeArmTargetPosition) || (direction < 0 && nextPosition < intakeArmTargetPosition)) nextPosition = intakeArmTargetPosition;
        setIntakeArmPosition(nextPosition);
    }

    public void setOuttakeArmPosition(double position) {
        if (MathFunctions.roughlyEquals(leftOuttakeArm.getPosition(), position)) {
            leftOuttakeArm.setPosition(position);
            rightOuttakeArm.setPosition(1 - position + RIGHT_OUTTAKE_ARM_OFFSET);
            updateOuttakeWrist();
        }
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
        if (!MathFunctions.roughlyEquals(outtakeWrist.getPosition(), position)) outtakeWrist.setPosition(position);
    }

    public void updateFrameTime() {
        deltaTimeSeconds = frameTimer.getElapsedTimeSeconds();
        frameTimer.resetTimer();
    }
}