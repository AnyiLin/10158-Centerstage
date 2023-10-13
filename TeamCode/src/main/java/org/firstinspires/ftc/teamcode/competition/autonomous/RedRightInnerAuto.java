package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.competition.teleop.TwoPersonDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.TeamPropPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Right Inner Auto", group = "Autonomous")
public class RedRightInnerAuto extends LinearOpMode {

    private TwoPersonDrive twoPersonDrive = new TwoPersonDrive();

    private DcMotorEx leftFront = twoPersonDrive.leftFront, leftRear = twoPersonDrive.leftRear, rightFront = twoPersonDrive.rightFront, rightRear = twoPersonDrive.rightRear, leftLift = twoPersonDrive.leftFront, rightLift = twoPersonDrive.rightLift, intake = twoPersonDrive.intake;

    private Servo leftIntake = twoPersonDrive.leftIntake, rightIntake = twoPersonDrive.rightIntake, leftOuttake = twoPersonDrive.leftOuttake, rightOuttake = twoPersonDrive.rightOuttake, outerClaw = twoPersonDrive.outerClaw, innerClaw = twoPersonDrive.innerClaw;

    private final int
            LIFT_VELOCITY = RobotConstants.LIFT_VELOCITY,
            LIFT_GRAB_TOLERANCE = RobotConstants.LIFT_GRAB_TOLERANCE,
            LIFT_VELOCITY_TOLERANCE = RobotConstants.LIFT_VELOCITY_TOLERANCE,
            FINE_ADJUST_LIFT_CHANGE = 300, // this is set to encoder ticks/second
            REGULAR_LIFT_CHANGE = 2*620, // this is set to encoder ticks/second
            LIFT_MAX = RobotConstants.LIFT_MAX,
            LIFT_TOLERANCE = RobotConstants.LIFT_TOLERANCE,
            TOP_LINE_POSITION = RobotConstants.TOP_LINE_POSITION,
            MIDDLE_LINE_POSITION = RobotConstants.MIDDLE_LINE_POSITION,
            BOTTOM_LINE_POSITION = RobotConstants.BOTTOM_LINE_POSITION,
            LIFT_GRAB_POSITION = RobotConstants.LIFT_GRAB_POSITION,
            INTAKE_VELOCITY = RobotConstants.INTAKE_VELOCITY;

    private final double
            ROBOT_FRONT_LENGTH = RobotConstants.ROBOT_FRONT_LENGTH,
            ROBOT_BACK_LENGTH = RobotConstants.ROBOT_BACK_LENGTH,
            INTAKE_CHANGE = 40, // this is set to degrees/second
            OUTTAKE_CHANGE = 40, // this is set to degrees/second
            OUTTAKE_FINE_ADJUST_DEAD_ZONE = 0.8,
            RIGHT_INTAKE_OFFSET = RobotConstants.RIGHT_INTAKE_OFFSET,
            LEFT_INTAKE_OUT_POSITION = RobotConstants.LEFT_INTAKE_OUT_POSITION,
            RIGHT_INTAKE_OUT_POSITION = RobotConstants.RIGHT_INTAKE_OUT_POSITION,
            LEFT_INTAKE_OUT_PAUSE_POSITION = RobotConstants.LEFT_INTAKE_OUT_PAUSE_POSITION,
            RIGHT_INTAKE_OUT_PAUSE_POSITION = RobotConstants.RIGHT_INTAKE_OUT_PAUSE_POSITION,
            LEFT_INTAKE_IN_POSITION = RobotConstants.LEFT_INTAKE_IN_POSITION,
            RIGHT_INTAKE_IN_POSITION = RobotConstants.RIGHT_INTAKE_IN_POSITION,
            LEFT_INTAKE_MIDDLE_POSITION = RobotConstants.LEFT_INTAKE_MIDDLE_POSITION,
            RIGHT_INTAKE_MIDDLE_POSITION = RobotConstants.RIGHT_INTAKE_MIDDLE_POSITION,
            LEFT_INTAKE_DROP_POSITION = RobotConstants.LEFT_INTAKE_DROP_POSITION,
            RIGHT_INTAKE_DROP_POSITION = RobotConstants.RIGHT_INTAKE_DROP_POSITION,
            RIGHT_OUTTAKE_OFFSET = RobotConstants.RIGHT_OUTTAKE_OFFSET,
            LEFT_OUTTAKE_OUT_POSITION = RobotConstants.LEFT_OUTTAKE_OUT_POSITION,
            RIGHT_OUTTAKE_OUT_POSITION = RobotConstants.RIGHT_OUTTAKE_OUT_POSITION,
            LEFT_OUTTAKE_IN_POSITION= RobotConstants.LEFT_OUTTAKE_IN_POSITION,
            RIGHT_OUTTAKE_IN_POSITION = RobotConstants.RIGHT_OUTTAKE_IN_POSITION,
            LEFT_OUTTAKE_GRAB_POSITION = RobotConstants.LEFT_OUTTAKE_GRAB_POSITION,
            RIGHT_OUTTAKE_GRAB_POSITION = RobotConstants.RIGHT_OUTTAKE_GRAB_POSITION,
            LEFT_OUTTAKE_AVOID_POSITION = RobotConstants.LEFT_OUTTAKE_AVOID_POSITION,
            RIGHT_OUTTAKE_AVOID_POSITION = RobotConstants.RIGHT_OUTTAKE_AVOID_POSITION,
            OUTER_CLAW_CLOSE_POSITION = RobotConstants.OUTER_CLAW_CLOSE_POSITION,
            INNER_CLAW_CLOSE_POSITION = RobotConstants.INNER_CLAW_CLOSE_POSITION,
            OUTER_CLAW_OPEN_POSITION = RobotConstants.OUTER_CLAW_OPEN_POSITION,
            INNER_CLAW_OPEN_POSITION = RobotConstants.INNER_CLAW_OPEN_POSITION,
            INTAKE_SERVO_TO_DEGREES = RobotConstants.INTAKE_SERVO_TO_DEGREES,
            INTAKE_DEGREES_TO_SERVO = RobotConstants.INTAKE_DEGREES_TO_SERVO,
            OUTTAKE_SERVO_TO_DEGREES = RobotConstants.OUTTAKE_SERVO_TO_DEGREES,
            OUTTAKE_DEGREES_TO_SERVO = RobotConstants.OUTTAKE_DEGREES_TO_SERVO,
            OUTTAKE_PICK_UP_DEGREES_PER_SECOND = RobotConstants.OUTTAKE_PICK_UP_DEGREES_PER_SECOND;

    private final long
            LIFT_GRAB_TIMEOUT = RobotConstants.LIFT_GRAB_TIMEOUT,
            INTAKE_IN_WAIT = RobotConstants.INTAKE_IN_WAIT,
            INTAKE_OBSTACLE_OUT_WAIT = RobotConstants.INTAKE_OBSTACLE_OUT_WAIT,
            INTAKE_OBSTACLE_OUT_RETRACT_WAIT = RobotConstants.INTAKE_OBSTACLE_OUT_RETRACT_WAIT,
            INTAKE_OBSTACLE_IN_WAIT = RobotConstants.INTAKE_OBSTACLE_IN_WAIT,
            OUTTAKE_OBSTACLE_FOLD_IN_WAIT = RobotConstants.OUTTAKE_OBSTACLE_FOLD_IN_WAIT,
            CLAW_GRAB_WAIT = RobotConstants.CLAW_GRAB_WAIT,
            CLAW_CLOSE_WAIT = RobotConstants.CLAW_CLOSE_WAIT,
            CLAW_LIFT_WAIT = RobotConstants.CLAW_LIFT_WAIT,
            PRESET_TIMEOUT = RobotConstants.PRESET_TIMEOUT,
            RESET_PIXEL_DROP_WAIT = RobotConstants.RESET_PIXEL_DROP_WAIT,
            RESET_FOLD_IN_WAIT = RobotConstants.RESET_FOLD_IN_WAIT,
            LIFT_GO_WAIT = RobotConstants.LIFT_GO_WAIT,
            INTAKE_FULL_OUT_WAIT = RobotConstants.INTAKE_FULL_OUT_WAIT;

    private final PIDFCoefficients
            LIFT_UP_VELOCITY_PIDF_COEFFICIENTS = RobotConstants.LIFT_UP_VELOCITY_PIDF_COEFFICIENTS,
            LIFT_UP_POSITION_PIDF_COEFFICIENTS = RobotConstants.LIFT_UP_POSITION_PIDF_COEFFICIENTS,
            LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS = RobotConstants.LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS,
            LIFT_DOWN_POSITION_PIDF_COEFFICIENTS = RobotConstants.LIFT_DOWN_POSITION_PIDF_COEFFICIENTS;

    private boolean outtakeSlowPickup = twoPersonDrive.outtakeSlowPickup, outerClawButtonPressed = twoPersonDrive.outerClawButtonPressed, innerClawButtonPressed = twoPersonDrive.innerClawButtonPressed, liftGoing = twoPersonDrive.liftGoing, resetFoldIn = twoPersonDrive.resetFoldIn, resetPixelDrop = twoPersonDrive.resetPixelDrop, liftInGrabbingPosition = twoPersonDrive.liftInGrabbingPosition, resetInMotion = twoPersonDrive.resetInMotion, presetLifting = twoPersonDrive.presetLifting, clawLifting = twoPersonDrive.clawLifting, clawClosing = twoPersonDrive.clawClosing, outtakeIn = twoPersonDrive.outtakeIn, intakeIn = twoPersonDrive.intakeIn, intakeGoingOut = twoPersonDrive.intakeGoingOut, intakeGoingInObstacle = twoPersonDrive.intakeGoingInObstacle, intakeGoingInObstacleFoldUp = twoPersonDrive.intakeGoingInObstacleFoldUp, intakeGoingIn = twoPersonDrive.intakeGoingIn, intakeGoingOutObstacle = twoPersonDrive.intakeGoingOutObstacle, intakeGoingOutObstacleRetract = twoPersonDrive.intakeGoingOutObstacleRetract, clawGrabbing = twoPersonDrive.clawGrabbing, presetInMotion = twoPersonDrive.presetInMotion, presetQueue = twoPersonDrive.presetQueue;

    private int leftLiftTargetPosition = twoPersonDrive.leftLiftTargetPosition, rightLiftTargetPosition = twoPersonDrive.rightLiftTargetPosition, presetTargetPosition = twoPersonDrive.presetTargetPosition, leftLiftCurrentAdjust = twoPersonDrive.leftLiftCurrentAdjust, rightLiftCurrentAdjust = twoPersonDrive.rightLiftCurrentAdjust;

    private long liftGrabStartTime = twoPersonDrive.liftGrabStartTime, resetFoldInStartTime = twoPersonDrive.resetFoldInStartTime, resetPixelDropStartTime = twoPersonDrive.resetPixelDropStartTime, presetStartTime = twoPersonDrive.presetStartTime, clawGrabbingStartTime = twoPersonDrive.clawGrabbingStartTime, intakeOutStartTime = twoPersonDrive.intakeOutStartTime, intakeInObstacleStartTime = twoPersonDrive.intakeInObstacleStartTime, intakeInStartTime = twoPersonDrive.intakeInStartTime, intakeOutObstacleStartTime = twoPersonDrive.intakeOutObstacleStartTime, lastFrameTimeNano = twoPersonDrive.lastFrameTimeNano, deltaTimeNano = twoPersonDrive.deltaTimeNano;

    private OpenCvCamera camera;

    private TeamPropPipeline teamPropPipeline;

    private String navigation;

    // all spike mark locations since I'm lazy
    private Pose2d redLeftSideLeftSpikeMark = new Pose2d(-47.5,-36);
    private Pose2d redLeftSideMiddleSpikeMark = new Pose2d(-36,-24.5);
    private Pose2d redLeftSideRightSpikeMark = new Pose2d(-24.5,-36);
    private Pose2d redRightSideLeftSpikeMark = new Pose2d(0.5,-36);
    private Pose2d redRightSideMiddleSpikeMark = new Pose2d(12,-24.5);
    private Pose2d redRightSideRightSpikeMark = new Pose2d(23.5,-36);
    private Pose2d blueLeftSideLeftSpikeMark = new Pose2d(23.5,36);
    private Pose2d blueLeftSideMiddleSpikeMark = new Pose2d(12,24.5);
    private Pose2d blueLeftSideRightSpikeMark = new Pose2d(0.5,36);
    private Pose2d blueRightSideLeftSpikeMark = new Pose2d(-24.5,36);
    private Pose2d blueRightSideMiddleSpikeMark = new Pose2d(-36,24.5);
    private Pose2d blueRightSideRightSpikeMark = new Pose2d(-47.5,36);

    // backdrop april tag locations
    private Pose2d blueLeftBackdrop = new Pose2d(60.75, 72-22.5-6.625);
    private Pose2d blueMiddleBackdrop = new Pose2d(60.75, 72-22.5-12.75);
    private Pose2d blueRightBackdrop = new Pose2d(60.75, 72-22.5-18.75);
    private Pose2d redLeftBackdrop = new Pose2d(60.75, -72+22.5+18.75);
    private Pose2d redMiddleBackdrop = new Pose2d(60.75, -72+22.5+12.75);
    private Pose2d redRightBackdrop = new Pose2d(60.75, -72+22.5+6.625);

    private Pose2d spikeMarkGoalPose, backdropGoalPose;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(12,-72+8.25, Math.toRadians(0)); // TODO: angle may or may not be right

    private SampleMecanumDrive drive;

    private TrajectorySequence scoreSpikeMark;

    public void setBackdropGoalPose() {
        switch (navigation) {
            case "left":
                spikeMarkGoalPose = new Pose2d(redRightSideLeftSpikeMark.getX()+(ROBOT_FRONT_LENGTH/Math.sqrt(2)), redRightSideLeftSpikeMark.getY()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), Math.toRadians(135));
                backdropGoalPose = new Pose2d(redLeftBackdrop.getX()-ROBOT_BACK_LENGTH, redLeftBackdrop.getY(), Math.toRadians(180));
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(redRightSideMiddleSpikeMark.getX(), redRightSideMiddleSpikeMark.getY()-ROBOT_FRONT_LENGTH, Math.toRadians(90));
                backdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH, redMiddleBackdrop.getY(), Math.toRadians(180));
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(redRightSideRightSpikeMark.getX()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), redRightSideRightSpikeMark.getY()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), Math.toRadians(45));
                backdropGoalPose = new Pose2d(redRightBackdrop.getX()-ROBOT_BACK_LENGTH, redRightBackdrop.getY(), Math.toRadians(180));
                break;
        }
    }

    public void buildTrajectories() {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        // this does the scoring on the spike mark at the start of auto
        scoreSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> twoPersonDrive.innerClaw.setPosition(INNER_CLAW_CLOSE_POSITION)) // close inner claw
                .UNSTABLE_addTemporalMarkerOffset(0,()-> twoPersonDrive.outerClaw.setPosition(OUTER_CLAW_CLOSE_POSITION)) // close outer claw
                .lineToConstantHeading(new Vector2d(12, -36-11.5))
                .lineToLinearHeading(spikeMarkGoalPose) // goes to the red spike mark location
                /**.lineToConstantHeading(new Vector2d(12, -36-11.5))
                .splineTo(new Vector2d(40, -24), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0,()-> twoPersonDrive.startPreset(0))
                .splineToLinearHeading(backdropGoalPose, backdropGoalPose.getHeading())
                 **/
                .build();
    }

    public void initialize() {
        twoPersonDrive.hardwareMap = hardwareMap;

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

        twoPersonDrive.initialize();
    }

    public void autonomous() {
        drive.followTrajectorySequence(scoreSpikeMark );
    }

    @Override
    public void runOpMode() {

        initialize();

        while (!isStarted() && !isStopRequested()) {
            navigation = teamPropPipeline.getNavigation();
            telemetry.addData("Navigation:", navigation);
            telemetry.update();
        }
        camera.stopStreaming();
        setBackdropGoalPose();
        buildTrajectories();

        if (isStopRequested()) return;
        if (!isStopRequested()) autonomous(); // can't be too sure that we need to stop!

    }
}
