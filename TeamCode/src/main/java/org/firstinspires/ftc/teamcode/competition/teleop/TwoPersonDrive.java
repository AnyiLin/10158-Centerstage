package org.firstinspires.ftc.teamcode.competition.teleop;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_AVOID;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_AVOID_RESET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_NOMINAL;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_AVOID;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ENCODER_OUT_ANGLE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ENCODER_SAFE_ANGLE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_GOING_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_GOING_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_HIGH_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_LOW_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_MIDDLE_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_PRESET_HOLD_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_SERVO_TO_DEGREES;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_GOING_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_GOING_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_PRESET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WAIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WRIST_DEGREES_TO_SERVO;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_WRIST_VERTICAL_OFFSET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.PLANE_LAUNCHER_HOLD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_INTAKE_ARM_OFFSET;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.RIGHT_OUTTAKE_ARM_OFFSET;
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

    public SingleRunAction intakeArmIn, intakeArmOut, outtakeArmIn, outtakeArmOut, outtakeArmPreset, outtakeArmWait, extensionIn, extensionAvoid, liftZero, liftLowPreset, liftMiddlePreset, liftHighPreset;

    public Timer outtakeTimer;

    public NanoTimer frameTimer;

    public boolean autonomous;

    public long deltaTimeSeconds;

    public double outtakeWristDirection, intakeArmTargetPosition, outtakeArmTargetPosition, intakeArmOutPosition, outtakePreviousStaticPosition;

    public int intakeState, outtakeState, intakeArmTargetState, outtakeArmTargetState, liftTargetPosition, extensionTargetPosition, extensionState;

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
        frameTimer = new NanoTimer();


        follower = new Follower(hardwareMap, false);
        driveVector = new Vector();
        headingVector = new Vector();


        liftPIDF = new PIDFController(liftPIDFCoefficients);
        extensionPIDF = new PIDFController(extensionPIDFCoefficients);

        liftTargetPosition = 0;
        extensionTargetPosition = 0;

        extensionIn = new SingleRunAction(()-> setExtensionTargetPosition(0));
        extensionAvoid = new SingleRunAction(()-> setExtensionTargetPosition(EXTENSION_AVOID_POSITION));
        liftZero = new SingleRunAction(()-> setLiftTargetPosition(0));
        liftLowPreset = new SingleRunAction(()-> setLiftTargetPosition(LIFT_LOW_PRESET_POSITION));
        liftMiddlePreset = new SingleRunAction(()-> setLiftTargetPosition(LIFT_MIDDLE_PRESET_POSITION));
        liftHighPreset = new SingleRunAction(()-> setLiftTargetPosition(LIFT_HIGH_PRESET_POSITION));
        intakeArmIn = new SingleRunAction(()-> setIntakeArmPosition(INTAKE_ARM_IN_POSITION));
        intakeArmOut = new SingleRunAction(()-> {setIntakeArmPosition(intakeArmOutPosition); intakeArmOutPosition = INTAKE_ARM_OUT_POSITION;});
        outtakeArmIn = new SingleRunAction(()-> setOuttakeArmPosition(OUTTAKE_ARM_IN_POSITION));
        outtakeArmOut = new SingleRunAction(()-> setOuttakeArmPosition(OUTTAKE_ARM_OUT_POSITION));
        outtakeArmPreset = new SingleRunAction(()-> setOuttakeArmPosition(OUTTAKE_ARM_PRESET_HOLD_POSITION));
        outtakeArmWait = new SingleRunAction(()-> setOuttakeArmPosition(outtakePreviousStaticPosition));

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

            drive();
        }
    }

    public void drive() {
        driveVector.setOrthogonalComponents(-gamepad1.left_stick_y, gamepad1.right_stick_x);
        driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
        driveVector.rotateVector(follower.getPose().getHeading());

        headingVector.setComponents(-gamepad1.left_stick_x, follower.getPose().getHeading());

        follower.setMovementVectors(follower.getCorrectiveVector(), driveVector, headingVector);
        follower.update();
    }

    public void resetAllActions() {
        resetIntakeActions();
        resetOuttakeActions();
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
        switch (intakeArmTargetState) {
            case INTAKE_IN:
            case INTAKE_OUT:
                intakeArmTargetState = state;
                updateIntake();
                break;
        }
    }

    public void setIntakeState(int state) {
        switch (intakeState) {
            case INTAKE_IN:
            case INTAKE_OUT:
            case INTAKE_AVOID:
            case INTAKE_GOING_IN:
            case INTAKE_GOING_OUT:
                intakeState = state;
                resetIntakeActions();
                break;
        }
    }

    public void updateIntake() {
        switch (intakeArmTargetState) {
            case INTAKE_IN:
                switch (intakeState) {
                    case INTAKE_IN:
                        break;
                    case INTAKE_OUT:
                        if (outtakeState == OUTTAKE_GOING_IN || outtakeState == OUTTAKE_OUT) {
                            setIntakeState(INTAKE_AVOID);
                            updateIntake();
                        } else {
                            setIntakeState(INTAKE_GOING_IN);
                            updateIntake();
                        }
                        break;
                    case INTAKE_GOING_IN:
                        if (intakeArmTargetPosition != leftIntakeArm.getPosition()) {
                            setIntakeArmPosition(INTAKE_ARM_IN_POSITION);
                            intakeArmTargetPosition = leftIntakeArm.getPosition();
                        }
                        if (intakeIsFolded()) {
                            setIntakeState(INTAKE_IN);
                            updateIntake();
                        }
                        break;
                    case INTAKE_GOING_OUT:
                        setIntakeState(INTAKE_GOING_IN);
                        updateIntake();
                        break;
                    case INTAKE_AVOID:
                        if (extensionTargetPosition < EXTENSION_AVOID_POSITION) setExtensionState(EXTENSION_AVOID);
                        if (extensionEncoder.getCurrentPosition() > EXTENSION_AVOID_POSITION) {
                            setIntakeState(INTAKE_GOING_IN);
                            updateIntake();
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
                            outtakeArmWait.run();
                            setOuttakeState(OUTTAKE_WAIT);
                            setIntakeState(INTAKE_AVOID);
                            updateIntake();
                        } else {
                            setIntakeState(INTAKE_GOING_OUT);
                            updateIntake();
                        }
                        break;
                    case INTAKE_OUT:
                        break;
                    case INTAKE_GOING_IN:
                        setIntakeState(INTAKE_GOING_OUT);
                        updateIntake();
                        break;
                    case INTAKE_GOING_OUT:
                        intakeArmOut.run();
                        if (intakeIsOut()) {
                            setIntakeState(INTAKE_OUT);
                            updateOuttake();
                        }
                        break;
                    case INTAKE_AVOID:
                        if (outtakeArmWait.hasBeenRun()) outtakeArmWait.reset();
                        if (extensionTargetPosition < EXTENSION_AVOID_POSITION) setExtensionState(EXTENSION_AVOID);
                        if (extensionEncoder.getCurrentPosition() > EXTENSION_AVOID_POSITION) {
                            setIntakeState(INTAKE_GOING_OUT);
                            if (outtakePreviousStaticPosition == OUTTAKE_ARM_IN_POSITION) {
                                setOuttakeState(OUTTAKE_IN);
                                updateOuttake();
                            } else if (outtakePreviousStaticPosition == OUTTAKE_ARM_PRESET_HOLD_POSITION) {
                                setOuttakeState(OUTTAKE_PRESET);
                                updateOuttake();
                            }
                            updateIntake();
                        }
                        break;
                }
                break;
        }
    }

    public void moveOuttake(int state) {
        switch (outtakeState) {
            case OUTTAKE_IN:
            case OUTTAKE_PRESET:
            case OUTTAKE_OUT:
                outtakeArmTargetState = state;
                updateOuttake();
                break;
        }
    }

    public void setOuttakeState(int state) {
        switch (outtakeArmTargetState) {
            case OUTTAKE_IN:
            case OUTTAKE_PRESET:
            case OUTTAKE_OUT:
            case OUTTAKE_GOING_IN:
            case OUTTAKE_GOING_OUT:
            case OUTTAKE_WAIT:
                outtakeTimer.resetTimer();
                outtakeState = state;
                resetOuttakeActions();
                break;
        }
    }

    // TODO: whenever the outtake reaches a static state, check if extension is avoiding, and if it is, then return the extension
    public void updateOuttake() {
        //if (extensionState == EXTENSION_AVOID) setExtensionState(EXTENSION_AVOID_RESET);
        if (!(outtakeState == OUTTAKE_WAIT)) {
            switch (outtakeArmTargetState) {
                case OUTTAKE_IN:
                    switch (outtakeState) {
                        case OUTTAKE_IN:
                            outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
                            if (extensionState == EXTENSION_AVOID) setExtensionState(EXTENSION_AVOID_RESET);
                            break;
                        case OUTTAKE_PRESET:
                            break;
                        case OUTTAKE_OUT:
                            if ((intakeState == INTAKE_IN || intakeState == INTAKE_GOING_IN || intakeState == INTAKE_GOING_OUT) && (extensionEncoder.getCurrentPosition() < EXTENSION_AVOID_POSITION)) {
                            } else {
                                intakeState = INTAKE_GOING_IN;
                                updateIntake();
                            }
                            break;
                        case OUTTAKE_GOING_IN:
                            if (extensionAvoid.hasBeenRun()) extensionAvoid.reset();
                            if (intakeArmTargetPosition != leftIntakeArm.getPosition()) {
                                setIntakeArmPosition(INTAKE_ARM_IN_POSITION);
                                intakeArmTargetPosition = leftIntakeArm.getPosition();
                            }
                            break;
                        case OUTTAKE_GOING_OUT:
                            intakeState = INTAKE_GOING_IN;
                            updateIntake();
                            break;
                    }
                    break;
                case OUTTAKE_OUT:
                    switch (outtakeState) {
                        case OUTTAKE_IN:
                            if (!(outtakeState == OUTTAKE_GOING_IN || outtakeState == OUTTAKE_OUT)) {
                                // TODO: this means that the outtake is moving while the intake is in the way
                                // set the outtake to move back to its previous static state
                                // and push the intake to avoiding
                                // when the intake is out of the way, put the outtake in and then fold up
                                setOuttakeArmPosition(outtakePreviousStaticPosition);
                            } else {
                                // TODO: coast is clear, move on
                            }
                            break;
                        case OUTTAKE_PRESET:
                            break;
                        case OUTTAKE_OUT:
                            if (intakeArmOut.hasBeenRun()) intakeArmOut.reset();
                            break;

                        // TODO: fix up below

                        case OUTTAKE_GOING_IN:
                            intakeState = INTAKE_GOING_OUT;
                            updateIntake();
                            break;
                        case OUTTAKE_GOING_OUT:
                            if (extensionAvoid.hasBeenRun()) extensionAvoid.reset();
                            intakeArmOut.run();
                            // TODO: detect when the arm is in position and update the state to intake out
                            break;
                    }
                    break;
                case OUTTAKE_PRESET:
                    // TODO: essentially make this the same as outtake out except with slightly different timings
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
                setExtensionTargetPosition(EXTENSION_AVOID_POSITION);
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
