package org.firstinspires.ftc.teamcode.notcompetition.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
@TeleOp(name = "Test TeleOp Enhancements", group = "Test4")
public class TestTeleOpEnhancements extends OpMode {
    private Follower follower;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private Vector driveVector, headingVector;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, false);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        double throttle = 0.2 + 0.8 * gamepad1.right_trigger;

        double strafe = 0;
        if (gamepad1.left_bumper) {
            strafe += 1;
        }
        if (gamepad1.right_bumper) {
            strafe -= 1;
        }

        driveVector.setOrthogonalComponents(-gamepad1.left_stick_y * throttle, strafe * throttle);
        driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
        driveVector.rotateVector(follower.getPose().getHeading());


        double rx = 0;
        if (Math.abs(gamepad1.left_stick_x)>0.1) rx = -gamepad1.left_stick_x;
        if (rx > 1-gamepad1.right_trigger*0.5) {
            rx = 1-gamepad1.right_trigger*0.5;
        } else if (rx<-1+gamepad1.right_trigger*0.5) {
            rx = -1+gamepad1.right_trigger*0.5;
        }
        rx *= throttle;

        headingVector.setComponents(rx, follower.getPose().getHeading());

        follower.setMovementVectors(follower.getCentripetalForceCorrection(), headingVector, driveVector);
        follower.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
