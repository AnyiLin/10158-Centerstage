package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.NanoTimer;

public class ThreeWheelLocalizer extends Localizer {
    private HardwareMap hardwareMap;
    private Pose startPose;
    private Pose currentPose;
    private Pose currentVelocity;
    private Matrix startRotationMatrix;
    private NanoTimer timer;
    private long deltaTimeNano;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder strafeEncoder;
    private Pose leftEncoderPose;
    private Pose rightEncoderPose;
    private Pose strafeEncoderPose;

    public ThreeWheelLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public ThreeWheelLocalizer(HardwareMap map, Pose setStartPose) {
        // TODO: replace these with your encoder positions
        leftEncoderPose = new Pose(-18.5/25.4 - 0.1, 164.4/25.4, 0);
        rightEncoderPose = new Pose(-18.4/25.4 - 0.1, -159.6/25.4, 0);
        strafeEncoderPose = new Pose(-107.9/25.4+0.25, -1.1/25.4-0.23, Math.toRadians(90));

        hardwareMap = map;

        // TODO: replace these with your encoder ports
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "strafeEncoder"));

        // TODO: reverse any encoders necessary
        leftEncoder.setDirection(Encoder.REVERSE);
        rightEncoder.setDirection(Encoder.REVERSE);
        strafeEncoder.setDirection(Encoder.FORWARD);

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
        // todo find delta positions from robot perspective
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
    }

    public void updateEncoders() {
        leftEncoder.update();
        rightEncoder.update();
        strafeEncoder.update();
    }

    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        // x/forward movement
        returnMatrix.set(0,0, (rightEncoder.getDeltaPosition() * leftEncoderPose.getY() - leftEncoder.getDeltaPosition() * rightEncoderPose.getY()) / (leftEncoderPose.getY() - rightEncoderPose.getY()));
        //y/strafe movement
        returnMatrix.set(1,0, strafeEncoder.getDeltaPosition() - strafeEncoderPose.getX() * ((rightEncoder.getDeltaPosition() - leftEncoder.getDeltaPosition()) / (leftEncoderPose.getY() - rightEncoderPose.getY())));
        // theta/turning
        returnMatrix.set(2,0, (rightEncoder.getDeltaPosition() - leftEncoder.getDeltaPosition()) / (leftEncoderPose.getY() - rightEncoderPose.getY()));
        return returnMatrix;
    }
}
