package org.firstinspires.ftc.teamcode.competition.teleop;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.EXTENSION_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_AVOID;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_GOING_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_GOING_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_SERVO_TO_DEGREES;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_GOING_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OUT;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@TeleOp(name = "Two Person Drive", group = "Drive")
public class TwoPersonDrive extends LinearOpMode {

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, leftExtension, rightExtension, liftEncoder, extensionEncoder;

    public Servo leftIntakeArm, rightIntakeArm, intakeClaw, leftOuttakeArm, rightOuttakeArm, outtakeWrist, outerOuttakeClaw, innerOuttakeClaw, plane;

    public Follower follower;
    public Vector driveVector, headingVector;

    public Telemetry telemetryA;

    public PIDFController liftPIDF = new PIDFController(liftPIDFCoefficients),
    extensionPIDF = new PIDFController(extensionPIDFCoefficients);

    public boolean autonomous;

    public long lastFrameTimeNano, deltaTimeNano, intakeArmMovementStartTime, outtakeArmMovementStartTime;

    public double outtakeWristDirection;

    public int intakeState, outtakeState, intakeTargetState, outtakeTargetState;

    public TwoPersonDrive() {
    }

    public TwoPersonDrive(boolean setAuto) {
        autonomous = setAuto;
    }

    public void initialize() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        follower = new Follower(hardwareMap, false);
        driveVector = new Vector();
        headingVector = new Vector();


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

        plane.setPosition(PLANE_LAUNCHER_HOLD);
        setOuttakeArmPosition(OUTTAKE_ARM_IN_POSITION);
        setIntakeArmPosition(INTAKE_ARM_IN_POSITION);
        outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
        innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_OPEN);
        intakeClaw.setPosition(INTAKE_CLAW_OPEN);
        setOuttakeWristDirection(0);

        intakeState = INTAKE_IN;
        outtakeState = OUTTAKE_IN;

        setEncoderMotors();
    }

    public void setEncoderMotors() {
        liftEncoder = rightLift;
        extensionEncoder = null; // TODO: set later
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

        lastFrameTimeNano = System.nanoTime();
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

    public void moveIntake(int state) {
        switch (intakeTargetState) {
            case INTAKE_IN:
            case INTAKE_OUT:
                intakeTargetState = state;
                updateIntake();
                break;
        }
    }

    public void updateIntake() {
        switch (intakeTargetState) {
            case INTAKE_IN:
                switch (intakeState) {
                    case INTAKE_OUT:
                        if (outtakeState == OUTTAKE_GOING_IN || outtakeState == OUTTAKE_OUT) {
                            intakeState = INTAKE_AVOID;
                            updateIntake();
                        } else {
                            intakeState = INTAKE_GOING_IN;
                            updateIntake();
                        }
                        break;
                    case INTAKE_GOING_IN:
                        setIntakeArmPosition(INTAKE_ARM_IN_POSITION);
                        // TODO: detect end
                        break;
                    case INTAKE_GOING_OUT:
                        intakeState = INTAKE_GOING_IN;
                        updateIntake();
                        break;
                    case INTAKE_AVOID:
                        if (extensionEncoder.getCurrentPosition() > EXTENSION_AVOID_POSITION) {
                            intakeState = INTAKE_GOING_IN;
                            updateIntake();
                        }
                        break;
                }
                break;
            case INTAKE_OUT:
                break;
        }
    }

    public void setIntakeArmPosition(double position) {
        leftIntakeArm.setPosition(position);
        rightIntakeArm.setPosition(1 - position + RIGHT_INTAKE_ARM_OFFSET);
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
        deltaTimeNano = System.nanoTime()- lastFrameTimeNano;
        lastFrameTimeNano = System.nanoTime();
    }
}
