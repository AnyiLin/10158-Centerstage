package org.firstinspires.ftc.teamcode.notcompetition.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

public class TestTeleOpEnhancements extends OpMode {
    private Follower follower;

    private Vector driveVector, headingVector;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, false);

        driveVector = new Vector();
        headingVector = new Vector();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        driveVector.setOrthogonalComponents(-gamepad1.left_stick_y, gamepad1.right_stick_x);
        driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
        driveVector.rotateVector(follower.getPose().getHeading());

        headingVector.setComponents(-gamepad1.left_stick_x, follower.getPose().getHeading());

        follower.setMovementVectors(follower.getCorrectiveVector(), driveVector, headingVector);
        follower.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
