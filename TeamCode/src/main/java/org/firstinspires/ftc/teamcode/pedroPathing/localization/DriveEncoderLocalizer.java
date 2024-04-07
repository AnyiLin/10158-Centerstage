package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.NanoTimer;

/**
 * This is the DriveEncoderLocalizer class.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
@Config
public class DriveEncoderLocalizer extends Localizer { // todo: make drive encoders work
    private HardwareMap hardwareMap;
    private Pose startPose;
    private Pose currentPose;
    private Pose currentVelocity;
    private Matrix startRotationMatrix;
    private NanoTimer timer;
    private long deltaTimeNano;
    private Encoder leftFront;
    private Encoder rightFront;
    private Encoder leftRear;
    private Encoder rightRear;
    private double totalHeading;
    public static double FORWARD_TICKS_TO_INCHES = 1;
    public static double STRAFE_TICKS_TO_INCHES = 1;
    public static double TURN_TICKS_TO_INCHES = 1;
    public static double ROBOT_WIDTH = 1;
    public static double ROBOT_LENGTH = 1;

    public DriveEncoderLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public DriveEncoderLocalizer(HardwareMap map, Pose setStartPose) {

        hardwareMap = map;

        // TODO: replace these with your encoder ports
        leftFront = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightFront = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        leftRear = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightRear = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));

        // TODO: reverse any encoders necessary
        leftFront.setDirection(Encoder.REVERSE);
        rightRear.setDirection(Encoder.REVERSE);
        leftRear.setDirection(Encoder.FORWARD);
        rightRear.setDirection(Encoder.FORWARD);

        setStartPose(setStartPose);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        currentPose = startPose;
        currentVelocity = new Pose();
    }

    @Override
    public Pose getPose() {
        return currentPose.copy();
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity.copy();
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
        setStartRotationMatrix(startPose.getHeading());
    }

    public void setStartRotationMatrix(double startHeading) {
        startRotationMatrix = new Matrix(3,3);
        startRotationMatrix.set(0, 0, Math.cos(startHeading));
        startRotationMatrix.set(0, 1, -Math.sin(startHeading));
        startRotationMatrix.set(1, 0, Math.sin(startHeading));
        startRotationMatrix.set(1, 1, -Math.cos(startHeading));
        startRotationMatrix.set(2, 2, 1.0);
    }

    @Override
    public void setPose(Pose setPose) {
        currentPose = setPose;
    }

    @Override
    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        updateEncoders();
        Matrix robotDeltas = getRobotDeltas();
        Matrix globalDeltas = new Matrix(3,1);

        Matrix transformation = new Matrix(3,3);
        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(2, 2, 1.0);
        } else {
            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(2, 2, 1.0);
        }

        globalDeltas = Matrix.multiply(Matrix.multiply(startRotationMatrix, transformation), robotDeltas);

        currentPose.add(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        currentVelocity = new Pose(globalDeltas.get(0, 0) / (deltaTimeNano * Math.pow(10.0, 9)), globalDeltas.get(1, 0) / (deltaTimeNano * Math.pow(10.0, 9)), globalDeltas.get(2, 0) / (deltaTimeNano * Math.pow(10.0, 9)));

        totalHeading += globalDeltas.get(2, 0);
    }

    public void updateEncoders() {
        leftFront.update();
        rightFront.update();
        leftRear.update();
        rightRear.update();
    }

    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        // x/forward movement
        returnMatrix.set(0,0, FORWARD_TICKS_TO_INCHES * (leftFront.getDeltaPosition() + rightFront.getDeltaPosition() + leftRear.getDeltaPosition() + rightRear.getDeltaPosition()));
        //y/strafe movement
        returnMatrix.set(1,0, STRAFE_TICKS_TO_INCHES * (-leftFront.getDeltaPosition() + rightFront.getDeltaPosition() + leftRear.getDeltaPosition() - rightRear.getDeltaPosition()));
        // theta/turning
        returnMatrix.set(2,0, TURN_TICKS_TO_INCHES * ((-leftFront.getDeltaPosition() + rightFront.getDeltaPosition() - leftRear.getDeltaPosition() + rightRear.getDeltaPosition()) / (ROBOT_WIDTH + ROBOT_LENGTH)));
        return returnMatrix;
    }

    public double getTotalHeading() {
        return totalHeading;
    }
}
