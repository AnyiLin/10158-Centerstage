package org.firstinspires.ftc.teamcode.competition.autonomous;


import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_STACK_TOP_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_BACK_LENGTH;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.competition.teleop.TwoPersonDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.VisionPortalTeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Right Inner Auto", group = "Autonomous")
public class RedRightInnerAuto extends OpMode {

    private TwoPersonDrive twoPersonDrive;

    private Timer pathTimer;

    private VisionPortalTeamPropPipeline teamPropPipeline;

    private VisionPortal visionPortal;

    private String navigation;

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

    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose, stackPose, firstCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(63+72, 12+72, Math.PI/2);

    private Follower follower;

    private Path placeHolderPath;

    private PathChain placeHolderPathChain;

    private final long placeHolderTime = 0; // todo set all these later

    private int pathState, EXTENSION_SPIKE_MARK_POSITION;

    // TODO: redo these goal poses with the new ones since auto structure has changed
    public void setBackdropGoalPose() {
        switch (navigation) {
            case "left":
                spikeMarkGoalPose = new Pose2d(redRightSideLeftSpikeMark.getX(), redRightSideLeftSpikeMark.getY(), Math.PI/2);
                EXTENSION_SPIKE_MARK_POSITION = 0;
                initialBackdropGoalPose = new Pose2d(redLeftBackdrop.getX(), +redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(redRightBackdrop.getX(), redRightBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                stackPose = new Pose2d(redInnerStack.getX(), redInnerStack.getY());
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(redRightSideMiddleSpikeMark.getX(), redRightSideMiddleSpikeMark.getY(), Math.PI/2);
                EXTENSION_SPIKE_MARK_POSITION = 0;
                initialBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX(), redMiddleBackdrop.getY()-ROBOT_BACK_LENGTH,Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(redLeftBackdrop.getX(), redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                stackPose = new Pose2d(redInnerStack.getX(), redInnerStack.getY());
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(redRightSideRightSpikeMark.getX(), redRightSideRightSpikeMark.getY(), Math.PI/2);
                EXTENSION_SPIKE_MARK_POSITION = 0;
                initialBackdropGoalPose = new Pose2d(redRightBackdrop.getX(), redRightBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(redLeftBackdrop.getX(), redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                stackPose = new Pose2d(redInnerStack.getX(), redInnerStack.getY());
                break;
        }
    }

    public void buildPaths() {

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // starts following the first path to score on the spike mark
                follower.followPath(new Path(null)); // todo replace with starting path
                twoPersonDrive.moveIntake(INTAKE_OUT);
                twoPersonDrive.setExtensionTargetPosition(EXTENSION_SPIKE_MARK_POSITION);
                setPathState(1);
                break;
            case 1: // detects for the end of the path and everything else to be in order and releases the pixel
                if (!follower.isBusy() && twoPersonDrive.intakeState == INTAKE_OUT && twoPersonDrive.extensionEncoder.getCurrentPosition() > EXTENSION_SPIKE_MARK_POSITION - 20) {
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_OPEN);
                    setPathState(2);
                }
                break;
            case 2: // moves mechanisms into position to score and pick up from stack as well as starts moving to score
                if (pathTimer.getElapsedTime() > 100) {
                    twoPersonDrive.setIntakeArmPosition(INTAKE_ARM_STACK_TOP_POSITION);
                    twoPersonDrive.moveOuttake(OUTTAKE_OUT);
                    follower.followPath(new Path(null)); // todo replace with scoring path
                    setPathState(3);
                }
                break;
            case 3: // detects for end of the path and outtake out and drops pixel
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
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void init() {
        twoPersonDrive = new TwoPersonDrive(true);
        twoPersonDrive.hardwareMap = hardwareMap;

        pathTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        teamPropPipeline = new VisionPortalTeamPropPipeline(0);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessors(teamPropPipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        twoPersonDrive.initialize();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        navigation = teamPropPipeline.getNavigation();
        telemetry.addData("Navigation:", navigation);
        telemetry.update();
        setBackdropGoalPose();
        buildPaths();

        if (gamepad1.a || gamepad1.cross) twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);

        if (gamepad1.b || gamepad1.circle) twoPersonDrive.innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_CLOSED);
    }

    @Override
    public void start() {
        super.start();
        visionPortal.stopStreaming();
        twoPersonDrive.frameTimer.resetTimer();
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