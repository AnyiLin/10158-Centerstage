package org.firstinspires.ftc.teamcode.competition.autonomous;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_FRONT_LENGTH;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.competition.teleop.TwoPersonDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.VisionPortalStackRelocalization;
import org.firstinspires.ftc.teamcode.util.VisionPortalTeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Dumb Red Right Outer Auto", group = "Autonomous")
public class DumbRedRightOuterAuto extends OpMode {

    private TwoPersonDrive twoPersonDrive = new TwoPersonDrive(true);

    private final PIDFCoefficients
            LIFT_UP_VELOCITY_PIDF_COEFFICIENTS = RobotConstants.LIFT_UP_VELOCITY_PIDF_COEFFICIENTS,
            LIFT_UP_POSITION_PIDF_COEFFICIENTS = RobotConstants.LIFT_UP_POSITION_PIDF_COEFFICIENTS,
            LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS = RobotConstants.LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS,
            LIFT_DOWN_POSITION_PIDF_COEFFICIENTS = RobotConstants.LIFT_DOWN_POSITION_PIDF_COEFFICIENTS;

    private VisionPortalTeamPropPipeline teamPropPipeline = new VisionPortalTeamPropPipeline(0);

    private VisionPortal visionPortal;

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

    // white pixel stack locations
    private Pose2d redOuterStack = new Pose2d(-72, -72+36);
    private Pose2d redMiddleStack = new Pose2d(-72, -72+48);
    private Pose2d redInnerStack = new Pose2d(-72, -72+60);
    private Pose2d blueInnerStack = new Pose2d(-72, 72-60);
    private Pose2d blueMiddleStack = new Pose2d(-72, 72-48);
    private Pose2d blueOuterStack = new Pose2d(-72, 72-36);

    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(-36,-72+9, Math.toRadians(90));

    private SampleMecanumDrive drive;

    private TrajectorySequence scoreSpikeMark, park;

    // these are auto specific timings and booleans
    private boolean atBackdrop, runningTrajectory;

    private long initializationSlideResetStartTime, atBackdropStartTime;

    private final long BACKDROP_WAIT_TIME = 0*1000, SCORE_WAIT_TIME = 500;

    private int trajectoryNumber;

    public void setBackdropGoalPose() {
        switch (navigation) {
            case "left":
                spikeMarkGoalPose = new Pose2d(redLeftSideLeftSpikeMark.getX()+(ROBOT_FRONT_LENGTH/Math.sqrt(2)), redLeftSideLeftSpikeMark.getY()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), Math.toRadians(135));
                initialBackdropGoalPose = new Pose2d(redLeftBackdrop.getX()-ROBOT_BACK_LENGTH, 1.5-2.5+redLeftBackdrop.getY(), Math.toRadians(180));
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(redLeftSideMiddleSpikeMark.getX(), redLeftSideMiddleSpikeMark.getY()-ROBOT_FRONT_LENGTH-1.25, Math.toRadians(90));
                initialBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH, -2+redMiddleBackdrop.getY(), Math.toRadians(180));
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(redLeftSideRightSpikeMark.getX()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), redLeftSideRightSpikeMark.getY()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), Math.toRadians(45));
                initialBackdropGoalPose = new Pose2d(redRightBackdrop.getX()-ROBOT_BACK_LENGTH, -1.5+redRightBackdrop.getY(), Math.toRadians(180));
                break;
        }
    }

    public void buildTrajectories() {

        // this does the scoring on the spike mark at the start of auto
        scoreSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .splineTo(new Vector2d(-36,-48), Math.toRadians(90))
                .splineToSplineHeading(spikeMarkGoalPose, spikeMarkGoalPose.getHeading())
                .lineToConstantHeading(new Vector2d(-36,-48))
                .lineToLinearHeading(new Pose2d(-39, -60, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, -60, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0,()-> twoPersonDrive.startPreset(0, false))
                .lineToLinearHeading(new Pose2d(40, initialBackdropGoalPose.getY(), Math.toRadians(180)))
                .lineToLinearHeading(initialBackdropGoalPose)
                .build();

        park = drive.trajectorySequenceBuilder(scoreSpikeMark.end())
                .lineToConstantHeading(new Vector2d(46, redMiddleBackdrop.getY()))
                .build();
    }

    public void initialize() {
        twoPersonDrive.hardwareMap = hardwareMap;

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessors(teamPropPipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        twoPersonDrive.initialize();
        twoPersonDrive.adjustingLiftZero = true;
    }

    @Override
    public void loop() {
        drive.update();
        twoPersonDrive.updateFrameTime();
        twoPersonDrive.asyncTimers();
        if (!twoPersonDrive.adjustingLiftZero) twoPersonDrive.updateLiftMotors();
        if (twoPersonDrive.presetInMotion) twoPersonDrive.detectPresetEnd();

        if (!drive.isBusy() && runningTrajectory) {
            runningTrajectory = false;
            trajectoryNumber++;
        }

        if (!drive.isBusy() && trajectoryNumber == 1 && !atBackdrop) {
            atBackdrop = true;
            atBackdropStartTime = System.currentTimeMillis();
        }
        if (trajectoryNumber == 1 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > BACKDROP_WAIT_TIME) {
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }
        if (trajectoryNumber == 1 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > BACKDROP_WAIT_TIME+SCORE_WAIT_TIME) {
            // after scoring on the backdrop, do something else
            runningTrajectory = true;
            drive.followTrajectorySequenceAsync(park);
            atBackdrop = false;
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }

        if (!drive.isBusy() && trajectoryNumber == 2 && !atBackdrop) {
            requestOpModeStop();
        }

        telemetry.addData("trajectory #", trajectoryNumber);
        telemetry.update();
    }

    @Override
    public void init() {
        initialize();
        twoPersonDrive.adjustingLiftZero = true;
        initializationSlideResetStartTime = System.currentTimeMillis();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        if (System.currentTimeMillis()-initializationSlideResetStartTime>1500) twoPersonDrive.asyncTimers();
        navigation = teamPropPipeline.getNavigation();
        telemetry.addData("Navigation:", navigation);
        telemetry.update();
        setBackdropGoalPose();
        buildTrajectories();

    }

    @Override
    public void start() {
        super.start();
        visionPortal.stopStreaming();
        drive.followTrajectorySequenceAsync(scoreSpikeMark);
        runningTrajectory = true;
        twoPersonDrive.lastFrameTimeNano = System.nanoTime();
    }

    @Override
    public void stop() {
        super.stop();
    }
}

/**
 * 8==D
 */
