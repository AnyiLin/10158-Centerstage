package org.firstinspires.ftc.teamcode.competition.teleop;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_AVOID;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_AVOID_RESET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_NOMINAL;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_AVOID;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ENCODER_OUT_ANGLE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ENCODER_SAFE_ANGLE;
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
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_OUT_IN_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_OUT_PRESET_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_PRESET_HOLD_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_PRESET_IN_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_SERVO_TO_DEGREES;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CLAW_CLOSE_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CLAW_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_GOING_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_GOING_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IDLE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_MOVING_OUTSIDE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_PRESET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WRIST_DEGREES_TO_SERVO;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WRIST_VERTICAL_OFFSET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.PLANE_LAUNCHER_HOLD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_INTAKE_ARM_OFFSET;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.util.NanoTimer;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.SingleRunAction;
import org.firstinspires.ftc.teamcode.util.Timer;

@TeleOp(name = "Two Person Drive", group = "Drive")
public class TwoPersonDrive extends LinearOpMode {

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, leftExtension, rightExtension, liftEncoder, extensionEncoder;

    public Servo leftIntakeArm, rightIntakeArm, intakeClaw, leftOuttakeArm, rightOuttakeArm, outtakeWrist, outerOuttakeClaw, innerOuttakeClaw, plane;

    public AnalogInput intakeArmInput;

    public Follower follower;
    public Vector driveVector, headingVector;

    public Telemetry telemetryA;

    public PIDFController liftPIDF, extensionPIDF;

    public SingleRunAction intakeArmIn, intakeArmOut, outtakeArmIn, outtakeArmOut, outtakeArmPreset, outtakeArmWait, extensionIn, extensionAvoid, liftManualControlReset, startTransfer, highPreset, middlePreset, lowPreset, resetPreset, intakeClawOpen, outtakeClawsClose, transferPresetHold, putOuttakeOut, outtakeClawsOpen, transferReset;

    public Timer outtakeTimer, transferTimer;

    public NanoTimer frameTimer;

    public boolean autonomous;

    public long deltaTimeSeconds, outtakeMovementTime;

    public double outtakeWristDirection, intakeArmTargetPosition, outtakeArmTargetPosition, intakeArmOutPosition, outtakePreviousStaticPosition;

    public int intakeState, outtakeState, intakeArmTargetState, outtakeArmTargetState, liftTargetPosition, liftPresetTargetPosition, extensionTargetPosition, extensionState, transferState;

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
        leftExtension.setDirection(DcMotorSimple.Direction.REVERSE);
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
        frameTimer = new NanoTimer();


        follower = new Follower(hardwareMap, false);
        driveVector = new Vector();
        headingVector = new Vector();


        liftPIDF = new PIDFController(liftPIDFCoefficients);
        extensionPIDF = new PIDFController(extensionPIDFCoefficients);

        liftTargetPosition = 0;
        liftPresetTargetPosition = 0;
        extensionTargetPosition = 0;

        extensionIn = new SingleRunAction(()-> setExtensionTargetPosition(0));
        extensionAvoid = new SingleRunAction(()-> setExtensionTargetPosition(EXTENSION_AVOID_POSITION));
        liftManualControlReset = new SingleRunAction(()-> setLiftTargetPosition(liftEncoder.getCurrentPosition()));
        intakeArmIn = new SingleRunAction(()-> setIntakeArmPosition(INTAKE_ARM_IN_POSITION));
        intakeArmOut = new SingleRunAction(()-> {setIntakeArmPosition(intakeArmOutPosition); intakeArmOutPosition = INTAKE_ARM_OUT_POSITION;});
        outtakeArmIn = new SingleRunAction(()-> setOuttakeArmPosition(OUTTAKE_ARM_IN_POSITION));
        outtakeArmOut = new SingleRunAction(()-> setOuttakeArmPosition(OUTTAKE_ARM_OUT_POSITION));
        outtakeArmPreset = new SingleRunAction(()-> setOuttakeArmPosition(OUTTAKE_ARM_PRESET_HOLD_POSITION));
        outtakeArmWait = new SingleRunAction(()-> setOuttakeArmPosition(outtakePreviousStaticPosition));
        startTransfer = new SingleRunAction(()-> {
            setExtensionTargetPosition(0);
            setLiftTargetPosition(0);
            moveOuttake(OUTTAKE_IN);
            moveIntake(INTAKE_IN);
            innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_OPEN);
            outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
        });
        intakeClawOpen = new SingleRunAction(()-> intakeClaw.setPosition(INTAKE_CLAW_OPEN));
        outtakeClawsClose = new SingleRunAction(()-> {
            outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_CLOSED);
            innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_CLOSED);
        });
        transferPresetHold = new SingleRunAction(()-> {
            liftPresetTargetPosition = 0;
            moveOuttake(OUTTAKE_PRESET);
            setLiftTargetPosition(liftPresetTargetPosition);
        });
        putOuttakeOut = new SingleRunAction(()-> moveOuttake(OUTTAKE_OUT));
        outtakeClawsOpen = new SingleRunAction(()-> {
            outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
            innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_OPEN);
        });
        transferReset = new SingleRunAction(()-> {
            moveOuttake(OUTTAKE_IN);
            setLiftTargetPosition(0);
        });
        highPreset = new SingleRunAction(()-> {
            liftPresetTargetPosition = LIFT_HIGH_PRESET_POSITION;
            if (transferState == TRANSFER_IDLE) setTransferState(TRANSFER_POSITIONING);
            if (transferState == TRANSFER_PRESET_HOLD) setTransferState(TRANSFER_OUT);
        });
        middlePreset = new SingleRunAction(()-> {
            liftPresetTargetPosition = LIFT_MIDDLE_PRESET_POSITION;
            if (transferState == TRANSFER_IDLE) setTransferState(TRANSFER_POSITIONING);
            if (transferState == TRANSFER_PRESET_HOLD) setTransferState(TRANSFER_OUT);
        });
        lowPreset = new SingleRunAction(()-> {
            liftPresetTargetPosition = LIFT_LOW_PRESET_POSITION;
            if (transferState == TRANSFER_IDLE) setTransferState(TRANSFER_POSITIONING);
            if (transferState == TRANSFER_PRESET_HOLD) setTransferState(TRANSFER_OUT);
        });
        resetPreset = new SingleRunAction(()-> {
            setTransferState(TRANSFER_RESET);
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
        intakeArmTargetPosition = leftIntakeArm.getPosition();
        outtakeArmTargetPosition = leftOuttakeArm.getPosition();
        outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
        intakeArmOutPosition = INTAKE_ARM_OUT_POSITION;
        extensionState = EXTENSION_NOMINAL;
        transferState = TRANSFER_IDLE;

        setEncoderMotors();
    }

    public void setEncoderMotors() {
        liftEncoder = rightLift;
        extensionEncoder = rightExtension;
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
        while (!isStarted() && !isStopRequested()) {
        }

        initialize();

        frameTimer.resetTimer();

        while (opModeIsActive()) {
            updateFrameTime();

            driverControls();
        }
    }

    public void driverControls() {
        drive();

        presetControls();

        teleopLiftControlUpdate();
    }

    public void drive() {
        driveVector.setOrthogonalComponents(-gamepad1.left_stick_y, -gamepad1.right_stick_x);
        driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
        driveVector.rotateVector(follower.getPose().getHeading());

        headingVector.setComponents(-gamepad1.left_stick_x, follower.getPose().getHeading());

        follower.setMovementVectors(follower.getCorrectiveVector(), driveVector, headingVector);
        follower.update();
    }

    public void presetControls() {
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

    public void teleopLiftControlUpdate() {// TODO: change this to the actual control yogi wants
        if ((outtakeState == OUTTAKE_OUT) && (Math.abs(gamepad2.left_stick_y) > 0) && ((liftEncoder.getCurrentPosition() < LIFT_MAX_POSITION && liftEncoder.getCurrentPosition() > 0) || (liftEncoder.getCurrentPosition() > LIFT_MAX_POSITION && -gamepad2.left_stick_y < 0) || (liftEncoder.getCurrentPosition() < 0 && -gamepad2.left_stick_y > 0))) {
            leftLift.setPower(-gamepad2.left_stick_y);
            rightLift.setPower(-gamepad2.right_stick_y);
            liftManualControlReset.reset();
        } else {
            updateLift();
            liftManualControlReset.run();
        }
    }

    public void setTransferState(int state) {
        switch (state) {
            case TRANSFER_IDLE:
            case TRANSFER_TRANSFERRING:
            case TRANSFER_POSITIONING:
            case TRANSFER_PRESET_HOLD:
            case TRANSFER_OUT:
            case TRANSFER_RESET:
            case TRANSFER_RESET_CLAW_DROP:
            case TRANSFER_GRAB:
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
        outtakeClawsClose.reset();
        transferPresetHold.reset();
        putOuttakeOut.reset();
        outtakeClawsOpen.reset();
        transferReset.reset();
    }

    public void updateTransfer() {
        if (!(transferState == TRANSFER_IDLE)) {
            switch (transferState) {
                case TRANSFER_POSITIONING:
                    if (intakeClaw.getPosition() == INTAKE_CLAW_OPEN) {
                        if (outerOuttakeClaw.getPosition() == OUTER_OUTTAKE_CLAW_OPEN || innerOuttakeClaw.getPosition() == INNER_OUTTAKE_CLAW_OPEN) {
                            setTransferState(TRANSFER_GRAB);
                        } else {
                            setTransferState(TRANSFER_OUT);
                        }
                        break;
                    }
                    startTransfer.run();
                    if (intakeState == INTAKE_IN && outtakeState == OUTTAKE_IN && extensionState == EXTENSION_NOMINAL && liftEncoder.getCurrentPosition() < LIFT_TRANSFER_UPPER_LIMIT) {
                        setTransferState(TRANSFER_PRESET_HOLD);
                    }
                    break;
                case TRANSFER_TRANSFERRING:
                    intakeClawOpen.run();
                    if (transferTimer.getElapsedTime() > TRANSFER_DROP_TIME) {
                        outtakeClawsClose.run();
                    }
                    if (transferTimer.getElapsedTime() > TRANSFER_DROP_TIME + OUTTAKE_CLAW_CLOSE_TIME) {
                        if (autonomous) {
                            setTransferState(TRANSFER_OUT);
                        } else {
                            setTransferState(TRANSFER_PRESET_HOLD);
                        }
                    }
                    break;
                case TRANSFER_GRAB:
                    outtakeClawsClose.run();
                    if (transferTimer.getElapsedTime() > OUTTAKE_CLAW_CLOSE_TIME) {
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
                    if (outerOuttakeClaw.getPosition() == OUTER_OUTTAKE_CLAW_CLOSED || innerOuttakeClaw.getPosition() == INNER_OUTTAKE_CLAW_CLOSED) {
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
        intakeArmIn.reset();
        intakeArmOut.reset();
    }

    public void resetOuttakeActions() {
        outtakeArmIn.reset();
        outtakeArmPreset.reset();
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
                            if (intakeArmTargetPosition != leftIntakeArm.getPosition()) {
                                setIntakeArmPosition(INTAKE_ARM_IN_POSITION);
                                intakeArmTargetPosition = leftIntakeArm.getPosition();
                            }
                            if (intakeIsFolded()) {
                                setIntakeState(INTAKE_IN);
                            }
                            break;
                        case INTAKE_GOING_OUT:
                            setIntakeState(INTAKE_GOING_IN);
                            break;
                        case INTAKE_AVOID:
                            if (extensionTargetPosition < EXTENSION_AVOID_POSITION) {
                                setExtensionState(EXTENSION_AVOID);
                            }
                            if (extensionEncoder.getCurrentPosition() > EXTENSION_AVOID_POSITION) {
                                setIntakeState(INTAKE_GOING_IN);
                            }
                            break;
                    }
                    break;
                case INTAKE_OUT:
                    switch (intakeState) {
                        case INTAKE_IN:
                            if (!(outtakeState == OUTTAKE_GOING_IN || outtakeState == OUTTAKE_OUT)) {
                                // set the outtake to move back to its previous static state
                                // and push the intake to avoiding
                                // when the intake is out of the way, put the outtake in and then fold up
                                setOuttakeState(OUTTAKE_WAIT);
                                setIntakeState(INTAKE_AVOID);
                            } else {
                                setIntakeState(INTAKE_GOING_OUT);
                            }
                            break;
                        case INTAKE_OUT:
                            break;
                        case INTAKE_GOING_IN:
                            setIntakeState(INTAKE_GOING_OUT);
                            break;
                        case INTAKE_GOING_OUT:
                            intakeArmOut.run();
                            if (intakeIsOut()) {
                                setIntakeState(INTAKE_OUT);
                            }
                            break;
                        case INTAKE_AVOID:
                            if (extensionTargetPosition < EXTENSION_AVOID_POSITION)
                                setExtensionState(EXTENSION_AVOID);
                            if (extensionEncoder.getCurrentPosition() > EXTENSION_AVOID_POSITION) {
                                if (outtakePreviousStaticPosition == OUTTAKE_ARM_IN_POSITION) {
                                    setOuttakeState(OUTTAKE_IN);
                                } else if (outtakePreviousStaticPosition == OUTTAKE_ARM_PRESET_HOLD_POSITION) {
                                    setOuttakeState(OUTTAKE_PRESET);
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
            case OUTTAKE_PRESET:
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
            case OUTTAKE_PRESET:
            case OUTTAKE_OUT:
            case OUTTAKE_GOING_IN:
            case OUTTAKE_GOING_OUT:
            case OUTTAKE_MOVING_OUTSIDE:
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

    // todo dont allow for fine adjust movement control in teleop until the arm is out
    // todo also make the wrist face the right directions
    public void updateOuttake() {
        if (!(outtakeState == OUTTAKE_WAIT || outtakeState == OUTTAKE_IDLE)) {
            switch (outtakeArmTargetState) {
                case OUTTAKE_IN:
                    switch (outtakeState) {
                        case OUTTAKE_IN:
                            outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
                            if (extensionState == EXTENSION_AVOID) setExtensionState(EXTENSION_AVOID_RESET);
                            break;
                        case OUTTAKE_PRESET:
                            outtakePreviousStaticPosition = OUTTAKE_ARM_PRESET_HOLD_POSITION;
                            if ((intakeState == INTAKE_IN || intakeState == INTAKE_GOING_IN || intakeState == INTAKE_GOING_OUT) && (extensionTargetPosition < EXTENSION_AVOID_POSITION)) {
                                setIntakeState(INTAKE_AVOID);
                            } else {
                                outtakeMovementTime = OUTTAKE_ARM_PRESET_IN_TIME;
                                setOuttakeState(OUTTAKE_GOING_IN);
                            }
                            break;
                        case OUTTAKE_OUT:
                            outtakePreviousStaticPosition = OUTTAKE_ARM_PRESET_HOLD_POSITION;
                            if ((intakeState == INTAKE_IN || intakeState == INTAKE_GOING_IN || intakeState == INTAKE_GOING_OUT) && (extensionTargetPosition < EXTENSION_AVOID_POSITION)) {
                                setIntakeState(INTAKE_AVOID);
                            } else {
                                outtakeMovementTime = OUTTAKE_ARM_OUT_IN_TIME;
                                setOuttakeState(OUTTAKE_GOING_IN);
                            }
                            break;
                        case OUTTAKE_GOING_IN:
                            outtakeArmIn.run();
                            if (outtakeTimer.getElapsedTime() > outtakeMovementTime) {
                                setOuttakeState(OUTTAKE_IN);
                            }
                            break;
                        case OUTTAKE_GOING_OUT:
                        case OUTTAKE_MOVING_OUTSIDE:
                            outtakeMovementTime = OUTTAKE_ARM_OUT_IN_TIME;
                            setOuttakeState(OUTTAKE_GOING_IN);
                            break;
                    }
                    break;
                case OUTTAKE_OUT:
                    switch (outtakeState) {
                        case OUTTAKE_IN:
                            outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
                            if ((intakeState == INTAKE_IN || intakeState == INTAKE_GOING_IN || intakeState == INTAKE_GOING_OUT) && (extensionTargetPosition < EXTENSION_AVOID_POSITION)) {
                                setIntakeState(INTAKE_AVOID);
                            } else {
                                outtakeMovementTime = OUTTAKE_ARM_OUT_IN_TIME;
                                setOuttakeState(OUTTAKE_GOING_OUT);
                            }
                            break;
                        case OUTTAKE_PRESET:
                            outtakePreviousStaticPosition = OUTTAKE_ARM_PRESET_HOLD_POSITION;
                            outtakeMovementTime = OUTTAKE_ARM_OUT_PRESET_TIME;
                            setOuttakeState(OUTTAKE_MOVING_OUTSIDE);
                            break;
                        case OUTTAKE_OUT:
                            outtakePreviousStaticPosition = OUTTAKE_ARM_PRESET_HOLD_POSITION;
                            if (extensionState == EXTENSION_AVOID) setExtensionState(EXTENSION_AVOID_RESET);
                            break;
                        case OUTTAKE_GOING_IN:
                            outtakeMovementTime = OUTTAKE_ARM_OUT_IN_TIME;
                            setOuttakeState(OUTTAKE_GOING_OUT);
                            break;
                        case OUTTAKE_GOING_OUT:
                        case OUTTAKE_MOVING_OUTSIDE:
                            outtakeArmOut.run();
                            if (outtakeTimer.getElapsedTime() > outtakeMovementTime) {
                                setOuttakeState(OUTTAKE_OUT);
                            }
                            break;
                    }
                    break;
                case OUTTAKE_PRESET:
                    switch (outtakeState) {
                        case OUTTAKE_IN:
                            outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
                            if ((intakeState == INTAKE_IN || intakeState == INTAKE_GOING_IN || intakeState == INTAKE_GOING_OUT) && (extensionTargetPosition < EXTENSION_AVOID_POSITION)) {
                                setIntakeState(INTAKE_AVOID);
                            } else {
                                outtakeMovementTime = OUTTAKE_ARM_PRESET_IN_TIME;
                                setOuttakeState(OUTTAKE_GOING_OUT);
                            }
                            break;
                        case OUTTAKE_PRESET:
                            outtakePreviousStaticPosition = OUTTAKE_ARM_PRESET_HOLD_POSITION;
                            if (extensionState == EXTENSION_AVOID) setExtensionState(EXTENSION_AVOID_RESET);
                            break;
                        case OUTTAKE_OUT:
                            outtakePreviousStaticPosition = OUTTAKE_ARM_PRESET_HOLD_POSITION;
                            outtakeMovementTime = OUTTAKE_ARM_OUT_PRESET_TIME;
                            setOuttakeState(OUTTAKE_MOVING_OUTSIDE);
                            break;
                        case OUTTAKE_GOING_IN:
                            outtakeMovementTime = OUTTAKE_ARM_OUT_IN_TIME;
                            setOuttakeState(OUTTAKE_GOING_OUT);
                            break;
                        case OUTTAKE_GOING_OUT:
                        case OUTTAKE_MOVING_OUTSIDE:
                            outtakeArmPreset.run();
                            if (outtakeTimer.getElapsedTime() > outtakeMovementTime) {
                                setOuttakeState(OUTTAKE_PRESET);
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
        // TODO: update lift feedforward equation if applicable
        double liftPower = liftPIDF.runPIDF();
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);
    }

    public void updateExtension() {
        extensionPIDF.updateError(extensionTargetPosition - extensionEncoder.getCurrentPosition());
        double extensionPower = extensionPIDF.runPIDF();
        leftExtension.setPower(extensionPower);
        rightExtension.setPower(extensionPower);
    }

    public void setLiftTargetPosition(int position) {
        liftTargetPosition = position;
        liftPIDF.reset();
        updateLift();
    }

    public void setExtensionTargetPosition(int position) {
        extensionTargetPosition = position;
        extensionPIDF.reset();
        updateExtension();
    }

    public void setExtensionState(int state) {
        switch (state) {
            case EXTENSION_NOMINAL:
                extensionState = state;
                updateExtension();
                break;
            case EXTENSION_AVOID:
                extensionState = state;
                setExtensionTargetPosition(EXTENSION_AVOID_POSITION+10);
                break;
            case EXTENSION_AVOID_RESET:
                extensionState = EXTENSION_NOMINAL;
                setExtensionTargetPosition(0);
                break;
        }
    }

    public void setIntakeArmPosition(double position) {
        leftIntakeArm.setPosition(position);
        rightIntakeArm.setPosition(1 - position + RIGHT_INTAKE_ARM_OFFSET);
    }

    public double getIntakeArmPosition() {
        return intakeArmInput.getVoltage() / 3.3 * 360;
    }

    /**
     * This returns if the intake arm has reached a position where it can safely fold under the
     * trusses and stage door.
     *
     * @return returns if it is ok to pull the extension back in
     */
    public boolean intakeIsFolded() {
        return getIntakeArmPosition() < INTAKE_ENCODER_SAFE_ANGLE; // TODO: check if direction is right
    }

    public boolean intakeIsOut() {
        return getIntakeArmPosition() > INTAKE_ENCODER_OUT_ANGLE; // TODO: check if direction is right
    }

    public void setOuttakeArmPosition(double position) {
        leftOuttakeArm.setPosition(position);
        rightOuttakeArm.setPosition(1 - position + RIGHT_OUTTAKE_ARM_OFFSET);
        updateOuttakeWrist();
    }

    /**
     * Sets what direction we want the outtake wrist to point. This will be an absolute direction,
     * so the outtake arm's current angle should not influence the direction the wrist points.
     * 0 is pointing directly upwards
     *
     * @param direction the direction the outtake wrist will point. This is an absolute direction
     */
    public void setOuttakeWristDirection(double direction) {
        outtakeWristDirection = direction;
        updateOuttakeWrist();
    }

    /**
     * Updates the wrist's position so it keeps its angle as the outtake arm moves
     */
    public void updateOuttakeWrist() {
        outtakeWrist.setPosition(OUTTAKE_WRIST_VERTICAL_OFFSET + (OUTTAKE_ARM_IN_POSITION - leftOuttakeArm.getPosition()) * OUTTAKE_ARM_SERVO_TO_DEGREES * OUTTAKE_WRIST_DEGREES_TO_SERVO);
    }

    public void updateFrameTime() {
        deltaTimeSeconds = frameTimer.getElapsedTimeSeconds();
        frameTimer.resetTimer();
    }
}
