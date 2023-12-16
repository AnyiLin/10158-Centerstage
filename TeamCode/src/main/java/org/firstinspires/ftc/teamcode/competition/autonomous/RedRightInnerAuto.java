package org.firstinspires.ftc.teamcode.competition.autonomous;

import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.ROBOT_FRONT_LENGTH;
import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.ROBOT_INTAKE_LENGTH;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.competition.teleop.TwoPersonDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.util.VisionPortalTeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Right Inner Auto", group = "Autonomous")
public class RedRightInnerAuto extends OpMode {

    private TwoPersonDrive twoPersonDrive = new TwoPersonDrive(true);

    private VisionPortalTeamPropPipeline teamPropPipeline = new VisionPortalTeamPropPipeline(0);

    private VisionPortal visionPortal;

    private String navigation;

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
    private Pose2d startPose = new Pose2d(63+72, 12+72, Math.PI);

    private Follower follower;

    private int pathNumber;

    public void setBackdropGoalPose() {
        switch (navigation) {
            case "left":
                spikeMarkGoalPose = new Pose2d(redRightSideLeftSpikeMark.getX()+(ROBOT_FRONT_LENGTH/Math.sqrt(2))-1.5, redRightSideLeftSpikeMark.getY());
                initialBackdropGoalPose = new Pose2d(redLeftBackdrop.getX()-ROBOT_BACK_LENGTH -0.5, 2.5-2.5+redLeftBackdrop.getY() -0.5, Math.toRadians(180));
                firstCycleBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5 -0.5, -1.5-2.5+redMiddleBackdrop.getY(), Math.toRadians(180));
                stackPose = new Pose2d(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH+1.5, redInnerStack.getY()-3);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(redRightSideMiddleSpikeMark.getX(), redRightSideMiddleSpikeMark.getY()-ROBOT_FRONT_LENGTH-1.25, Math.toRadians(90));
                initialBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH -0.5, -2+redMiddleBackdrop.getY() -0.5, Math.toRadians(180));
                firstCycleBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5 -0.5, -2+redLeftBackdrop.getY(), Math.toRadians(180));
                stackPose = new Pose2d(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH+0.75, redInnerStack.getY()-2);
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(redRightSideRightSpikeMark.getX()-(ROBOT_FRONT_LENGTH/Math.sqrt(2))+1.5, redRightSideRightSpikeMark.getY());
                initialBackdropGoalPose = new Pose2d(redRightBackdrop.getX()-ROBOT_BACK_LENGTH -0.5, -1.5+redRightBackdrop.getY() -0.25, Math.toRadians(180));
                firstCycleBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5 -0.5, -1.5+redMiddleBackdrop.getY(), Math.toRadians(180));
                stackPose = new Pose2d(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH+0.75, redInnerStack.getY()-2);
                break;
        }
    }

    public void buildPaths() {

    }

    public void initialize() {
        twoPersonDrive.hardwareMap = hardwareMap;

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

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
    public void loop() {
        follower.update();
        twoPersonDrive.updateFrameTime();

        if (!follower.isBusy() && pathNumber == 6) {
            requestOpModeStop();
        }

        telemetry.addData("path #", pathNumber);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void init() {
        initialize();
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
    }

    @Override
    public void stop() {
        super.stop();
    }
}

/**
 * 8==D
 */