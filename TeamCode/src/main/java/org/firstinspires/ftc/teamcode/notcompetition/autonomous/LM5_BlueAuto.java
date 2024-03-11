package org.firstinspires.ftc.teamcode.notcompetition.autonomous;


import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_STACK_MIDDLE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_STACK_TOP_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_MIDDLE_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_CLAW_DELAY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_RESET;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.competition.teleop.TwoPersonDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.SingleRunAction;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.VisionPortalTeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Autonomous(name = "LM5 Blue Auto", group = "Autonomous")
public class LM5_BlueAuto extends OpMode {

    private TwoPersonDrive twoPersonDrive;

    private Timer pathTimer, opmodeTimer;

    private VisionPortalTeamPropPipeline teamPropPipeline;

    private VisionPortal visionPortal;

    private String navigation;

    private SingleRunAction moveExtensionToSpikeMarkDrop, moveIntakeToTopStackPosition, moveIntakeToMiddleStackPosition, moveIntakeToAvoidBeaconPosition;

    // IMPORTANT: y increasing is towards the backstage from the audience,
    // while x increasing is towards the red side from the blue side
    // this means that 0 heading is pointing from the blue side to the red side

    // all spike mark locations since I'm lazy
    private Pose2d redLeftSideLeftSpikeMark = new Pose2d(36+72,-47.5+72);
    private Pose2d redLeftSideMiddleSpikeMark = new Pose2d(24.5+72,-36+72);
    private Pose2d redLeftSideRightSpikeMark = new Pose2d(36+72,-24.5+72);
    private Pose2d redRightSideLeftSpikeMark = new Pose2d(36+72, 0.5+72);
    private Pose2d redRightSideMiddleSpikeMark = new Pose2d(24.5+72, 12+72);
    private Pose2d redRightSideRightSpikeMark = new Pose2d(36+72, 23.5+72);
    private Pose2d blueLeftSideLeftSpikeMark = new Pose2d(-36+72, 23.5+72);
    private Pose2d blueLeftSideMiddleSpikeMark = new Pose2d(-24.5+72, 12+72);
    private Pose2d blueLeftSideRightSpikeMark = new Pose2d(-36+72, 0.5+72);
    private Pose2d blueRightSideLeftSpikeMark = new Pose2d(-36+72, -24.5+72);
    private Pose2d blueRightSideMiddleSpikeMark = new Pose2d(-24.5+72, -36+72);
    private Pose2d blueRightSideRightSpikeMark = new Pose2d(-36+72, -47.5+72);

    // backdrop april tag locations
    private Pose2d blueLeftBackdrop = new Pose2d(-42.875+72, 60.75+72);
    private Pose2d blueMiddleBackdrop = new Pose2d(-36.75+72, 60.75+72);
    private Pose2d blueRightBackdrop = new Pose2d(-30.75+72, 60.75+72);
    private Pose2d redLeftBackdrop = new Pose2d(30.75+72, 60.75+72);
    private Pose2d redMiddleBackdrop = new Pose2d(36.75+72, 60.75+72);
    private Pose2d redRightBackdrop = new Pose2d(42.875+72, 60.75+72);

    // white pixel stack locations
    private Pose2d redOuterStack = new Pose2d(36+72, -72+72);
    private Pose2d redMiddleStack = new Pose2d(24+72, -72+72);
    private Pose2d redInnerStack = new Pose2d(12+72,-72+72);
    private Pose2d blueInnerStack = new Pose2d(-12+72,-72+72);
    private Pose2d blueMiddleStack = new Pose2d(-24+72, -72+72);
    private Pose2d blueOuterStack = new Pose2d(-36+72, -72+72);

    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(144-(63+72), 12+72, 0);

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop, getOutOfWay;

    private double scoreSpikeMarkAngle;

    private int pathState, EXTENSION_SPIKE_MARK_POSITION;

    public void setBackdropGoalPose() {
        switch (navigation) {
            case "left":
                spikeMarkGoalPose = new Pose2d(blueLeftSideLeftSpikeMark.getX() + 10, blueLeftSideLeftSpikeMark.getY(), Math.PI/2);
                scoreSpikeMarkAngle = 6.048;
                EXTENSION_SPIKE_MARK_POSITION = 0;
                initialBackdropGoalPose = new Pose2d(blueLeftBackdrop.getX() - 1.25, blueLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(blueLeftSideMiddleSpikeMark.getX(), blueLeftSideMiddleSpikeMark.getY(), Math.PI/2);
                scoreSpikeMarkAngle = 5.422;
                EXTENSION_SPIKE_MARK_POSITION = 0;
                initialBackdropGoalPose = new Pose2d(blueMiddleBackdrop.getX() - 3.25, blueMiddleBackdrop.getY()-ROBOT_BACK_LENGTH,Math.PI * 1.5);
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(blueLeftSideRightSpikeMark.getX() + 8, blueLeftSideRightSpikeMark.getY(), Math.PI/2);
                scoreSpikeMarkAngle = 5.324;
                EXTENSION_SPIKE_MARK_POSITION = 0;
                initialBackdropGoalPose = new Pose2d(blueRightBackdrop.getX() - 2, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                break;
        }
    }

    public void buildPaths() {
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), new Point(144-123, 88, Point.CARTESIAN), new Point(144-121, 101, Point.CARTESIAN)));
        Vector scoreSpikeMarkVector = new Vector();
        scoreSpikeMarkVector.setOrthogonalComponents(spikeMarkGoalPose.getX() - scoreSpikeMark.getLastControlPoint().getX(), spikeMarkGoalPose.getY() - scoreSpikeMark.getLastControlPoint().getY());
        //scoreSpikeMarkAngle = scoreSpikeMarkVector.getTheta();
        scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());

        initialScoreOnBackdrop = new Path(new BezierCurve(new Point(scoreSpikeMark.getLastControlPoint().getX(), scoreSpikeMark.getLastControlPoint().getY(), Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 111, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
        initialScoreOnBackdrop.setConstantHeadingInterpolation(Math.PI * 1.5);
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(2.5);

        getOutOfWay = new Path(new BezierCurve(new Point(initialBackdropGoalPose), new Point(initialBackdropGoalPose.getX(), 106, Point.CARTESIAN), new Point(144-86, 120, Point.CARTESIAN)));
        getOutOfWay.setConstantHeadingInterpolation(Math.PI * 1.5);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // starts following the first path to score on the spike mark
                follower.followPath(scoreSpikeMark);
                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_OUT_POSITION+0.008);
                //twoPersonDrive.setExtensionTargetPosition(100);
                setPathState(1);
                break;
            case 1: // detects for the end of the path and everything else to be in order and releases the pixel
                if (follower.getCurrentTValue() > 0.6) scoreSpikeMark.setConstantHeadingInterpolation(scoreSpikeMarkAngle);
                if (follower.getCurrentTValue() > 0.8) moveExtensionToSpikeMarkDrop.run();
                if (!follower.isBusy() && twoPersonDrive.intakeState == INTAKE_OUT /*&& twoPersonDrive.extensionEncoder.getCurrentPosition() > EXTENSION_SPIKE_MARK_POSITION - 20*/) {
                    twoPersonDrive.setIntakeClawOpen(true);
                    setPathState(2);
                }
                break;
            case 2: // moves mechanisms into position to score and pick up from stack as well as starts moving to score
                if (pathTimer.getElapsedTime() > 100) {
                    //twoPersonDrive.setExtensionTargetPosition(0);
                    twoPersonDrive.moveOuttake(OUTTAKE_OUT);
                    twoPersonDrive.setLiftTargetPosition(LIFT_MIDDLE_PRESET_POSITION + 230);
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(3);
                }
                break;
            case 3: // detects for end of the path and outtake out and drops pixel
                if (pathTimer.getElapsedTime() > 400) {
                    moveIntakeToAvoidBeaconPosition.run();
                }
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    follower.holdPoint(new BezierPoint(new Point(initialScoreOnBackdrop.getLastControlPoint().getX(), initialScoreOnBackdrop.getLastControlPoint().getY() + 1.25, Point.CARTESIAN)), Math.PI * 1.5);
                    moveIntakeToAvoidBeaconPosition.reset();
                    setPathState(4);
                }
                break;
            case 4: // detects for end of the path and outtake out and drops pixel
                if (pathTimer.getElapsedTime() > 1500 && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    setPathState(5);
                }
                break;
            case 5: // detects for end of the path and outtake out and drops pixel
                if (pathTimer.getElapsedTime() > 800) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 700) {
                    twoPersonDrive.moveIntake(INTAKE_IN);
                    follower.followPath(getOutOfWay);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;



            case 8: // once the robot is nice and folded up, request stop
                if (twoPersonDrive.intakeState == INTAKE_IN && twoPersonDrive.outtakeState == OUTTAKE_IN/* && twoPersonDrive.extensionState == EXTENSION_NOMINAL*/) {
                    setPathState(-1);
                }
                break;


            default:
                requestOpModeStop();
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    @Override
    public void loop() {
        follower.update();
        twoPersonDrive.autonomousControlUpdate();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("GOAL HEADING", scoreSpikeMarkAngle);
        telemetry.addData("current HEADING", follower.getHeadingVector().getTheta());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void init() {
        //PhotonCore.start(this.hardwareMap);

        twoPersonDrive = new TwoPersonDrive(true);
        twoPersonDrive.hardwareMap = hardwareMap;

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.8);

        teamPropPipeline = new VisionPortalTeamPropPipeline(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessors(teamPropPipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        //moveExtensionToSpikeMarkDrop = new SingleRunAction(()-> twoPersonDrive.setExtensionTargetPosition(EXTENSION_SPIKE_MARK_POSITION));
        moveIntakeToTopStackPosition = new SingleRunAction(()-> twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_TOP_POSITION));
        moveIntakeToMiddleStackPosition = new SingleRunAction(()-> twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_MIDDLE_POSITION));
        //moveIntakeToAvoidBeaconPosition = new SingleRunAction(()-> twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_OUT_AUTO_AVOID_POSITION));

        twoPersonDrive.initialize();

        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);

        twoPersonDrive.innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_CLOSED);

        try {
            Thread.sleep(TRANSFER_CLAW_DELAY);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_CLOSED);
    }

    @Override
    public void init_loop() {
        super.init_loop();

        navigation = teamPropPipeline.getNavigation();
        telemetry.addData("Navigation:", navigation);
        telemetry.update();
        setBackdropGoalPose();
        buildPaths();
    }

    @Override
    public void start() {
        super.start();
        visionPortal.stopStreaming();
        twoPersonDrive.frameTimer.resetTimer();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        super.stop();
    }
}

/**
 * 8==D
 */