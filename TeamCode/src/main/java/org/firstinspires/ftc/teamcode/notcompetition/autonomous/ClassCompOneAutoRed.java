package org.firstinspires.ftc.teamcode.notcompetition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.opencv.TeamPropPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "Class Comp One Auto Red", group = "Class Comps")
public class ClassCompOneAutoRed extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private OpenCvCamera camera;

    private TeamPropPipeline teamPropPipeline;

    private String navigation;

    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void autonomous() {
        switch (navigation) {
            case "left":
                goForward(0.2, 1200);
                turn(0.2,700);
                goForward(0.2,950);
                goForward(-0.2, 950);
                turn (-0.2, 700);
                goForward(-0.2, 600);
                strafe(-0.4 ,2100);
                break;
            case "middle":
                goForward(0.2, 2475);
                goForward(-0.2, 1700);
                strafe(-0.4 ,1900);
                break;
            case "right":
                goForward(0.2, 1900);
                strafe(-0.3 ,950);
                goForward(-0.2, 1200);
                strafe(-0.4, 1550);
                break;
        }
    }

    public void goForward(double power, long time)  {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void turn(double power, long time) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftRear.setPower(-power);
        rightRear.setPower(power);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void strafe(double power, long time)  {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    @Override
    public void runOpMode() {

        initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        teamPropPipeline = new TeamPropPipeline(0);

        camera.setPipeline(teamPropPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                onOpened();
            }
        });
        while (!isStarted() && !isStopRequested()) {
            navigation = teamPropPipeline.getNavigation();
            telemetry.addData("Navigation:", navigation);
            telemetry.update();
        }
        camera.stopStreaming();

        if (isStopRequested()) return;
        if (!isStopRequested()) autonomous(); // can't be too sure that we need to stop!

    }
}
