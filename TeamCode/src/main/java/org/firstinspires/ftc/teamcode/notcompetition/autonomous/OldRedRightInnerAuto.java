package org.firstinspires.ftc.teamcode.notcompetition.autonomous;

import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.INNER_CLAW_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.INTAKE_STACK_TOP_POSITION;
import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.LEFT_OUTTAKE_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.LEFT_OUTTAKE_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.OUTER_CLAW_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.RIGHT_OUTTAKE_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.RIGHT_OUTTAKE_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.ROBOT_FRONT_LENGTH;
import static org.firstinspires.ftc.teamcode.util.OldRobotConstants.ROBOT_INTAKE_LENGTH;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.notcompetition.teleop.OldTwoPersonDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.OldRobotConstants;
import org.firstinspires.ftc.teamcode.util.VisionPortalStackRelocalization;
import org.firstinspires.ftc.teamcode.util.VisionPortalTeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Old Red Right Inner Auto", group = "Not Comp Autonomous")
public class OldRedRightInnerAuto extends OpMode {

    private OldTwoPersonDrive twoPersonDrive = new OldTwoPersonDrive(true);

    private final PIDFCoefficients
            LIFT_UP_VELOCITY_PIDF_COEFFICIENTS = OldRobotConstants.LIFT_UP_VELOCITY_PIDF_COEFFICIENTS,
            LIFT_UP_POSITION_PIDF_COEFFICIENTS = OldRobotConstants.LIFT_UP_POSITION_PIDF_COEFFICIENTS,
            LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS = OldRobotConstants.LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS,
            LIFT_DOWN_POSITION_PIDF_COEFFICIENTS = OldRobotConstants.LIFT_DOWN_POSITION_PIDF_COEFFICIENTS;

    private VisionPortalTeamPropPipeline teamPropPipeline = new VisionPortalTeamPropPipeline(0);

    private VisionPortalStackRelocalization stackRelocalization = new VisionPortalStackRelocalization();

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

    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose, firstStackPose, firstCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(12,-72+9, Math.toRadians(90));

    private SampleMecanumDrive drive;

    private TrajectorySequence scoreSpikeMark, getStackPixels, adjustStack, getStackPixels2, scoreFirstStackPixels, park;

    // these are auto specific timings and booleans
    private boolean scanningStack, scanningStack2, atBackdrop, runningTrajectory, atStack;

    private long scanningStackStartTime, scanningTime, initializationSlideResetStartTime, atBackdropStartTime, atStackStartTime;

    private final long BACKDROP_WAIT_TIME = 0*1000, SCANNING_TIME = 1000, SCORE_WAIT_TIME = 500, INTAKE_INTAKING_TIME = 2000;

    private int trajectoryNumber;

    public void setBackdropGoalPose() {
        switch (navigation) {
            case "left":
                spikeMarkGoalPose = new Pose2d(redRightSideLeftSpikeMark.getX()+(ROBOT_FRONT_LENGTH/Math.sqrt(2)), redRightSideLeftSpikeMark.getY()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), Math.toRadians(135));
                initialBackdropGoalPose = new Pose2d(redLeftBackdrop.getX()-ROBOT_BACK_LENGTH -0.5, 2.5-2.5+redLeftBackdrop.getY() -0.5, Math.toRadians(180));
                firstCycleBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5 -0.5, -1.5-2.5+redMiddleBackdrop.getY(), Math.toRadians(180));
                firstStackPose = new Pose2d(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH+0.75, redInnerStack.getY()-3);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(redRightSideMiddleSpikeMark.getX(), redRightSideMiddleSpikeMark.getY()-ROBOT_FRONT_LENGTH-1.25, Math.toRadians(90));
                initialBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH -0.5, -2+redMiddleBackdrop.getY() -0.5, Math.toRadians(180));
                firstCycleBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5 -0.5, -2+redLeftBackdrop.getY(), Math.toRadians(180));
                firstStackPose = new Pose2d(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH+0.75, redInnerStack.getY()-2);
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(redRightSideRightSpikeMark.getX()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), redRightSideRightSpikeMark.getY()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), Math.toRadians(45));
                initialBackdropGoalPose = new Pose2d(redRightBackdrop.getX()-ROBOT_BACK_LENGTH -0.5, -1.5+redRightBackdrop.getY() -0.25, Math.toRadians(180));
                firstCycleBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5 -0.5, -1.5+redMiddleBackdrop.getY(), Math.toRadians(180));
                firstStackPose = new Pose2d(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH+0.75, redInnerStack.getY()-2);
                break;
        }
    }

    public void buildTrajectories() {

        // this does the scoring on the spike mark at the start of auto
        scoreSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .splineTo(new Vector2d(12,-48), Math.toRadians(90))
                .splineToSplineHeading(spikeMarkGoalPose, spikeMarkGoalPose.getHeading())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15,-48), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0,()-> twoPersonDrive.startPreset(0, false))
                .splineToSplineHeading(new Pose2d(30, -56, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(35, initialBackdropGoalPose.getY(), Math.toRadians(180.00001)), Math.toRadians(90))
                .setReversed(false)
                .lineToLinearHeading(initialBackdropGoalPose)
                .resetConstraints()
                .build();

        getStackPixels = drive.trajectorySequenceBuilder(scoreSpikeMark.end())
                .splineToLinearHeading(new Pose2d(24, -12, Math.toRadians(180)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(firstStackPose.getX()+8, firstStackPose.getY()+0.0001, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()-> visionPortal.resumeStreaming())
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()-> visionPortal.setProcessorEnabled(teamPropPipeline, false))
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()-> visionPortal.setProcessorEnabled(stackRelocalization, true))
                .lineToLinearHeading(new Pose2d(firstStackPose.getX()+6, firstStackPose.getY(), Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0,()-> scanningTime = System.currentTimeMillis()-1000)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> scanningStackStartTime = System.currentTimeMillis())
                .UNSTABLE_addTemporalMarkerOffset(0,()-> scanningStack = true)
                .resetConstraints()
                .build();
    }

    public void adjustStack() {
        adjustStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(firstStackPose.getX()+6, firstStackPose.getY(), Math.toRadians(180)))
                .build();
    }

    public void stackAdjustTrajectories() {
        getStackPixels2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(firstStackPose.getX(), firstStackPose.getY(), Math.toRadians(180)))
                .resetConstraints()
                .build();

        scoreFirstStackPixels = drive.trajectorySequenceBuilder(getStackPixels2.end())
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(180)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(27, -12, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()-> twoPersonDrive.startPreset(50))
                .splineToLinearHeading(new Pose2d(40, firstCycleBackdropGoalPose.getY(), Math.toRadians(180)), Math.toRadians(180))
                .setReversed(false)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(firstCycleBackdropGoalPose)
                .resetConstraints()
                .build();

        park = drive.trajectorySequenceBuilder(scoreFirstStackPixels.end())
                .lineToConstantHeading(new Vector2d(46, redMiddleBackdrop.getY()))
                .build();
    }

    public void initialize() {
        twoPersonDrive.hardwareMap = hardwareMap;
        twoPersonDrive.burstIntake = false;

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessors(stackRelocalization, teamPropPipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        visionPortal.setProcessorEnabled(stackRelocalization, false);

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
            drive.followTrajectorySequenceAsync(getStackPixels);
            atBackdrop = false;
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }

        if (scanningStack) {
            if (System.currentTimeMillis()-scanningTime>700) {
                firstStackPose = new Pose2d(firstStackPose.getX(), drive.getPoseEstimate().getY()+stackRelocalization.getXErrorInches());
                adjustStack();
                drive.breakFollowing();
                drive.followTrajectorySequenceAsync(adjustStack);
                scanningTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis()-scanningStackStartTime>SCANNING_TIME) {
                scanningStack = false;
                scanningStack2 = true;
                stackAdjustTrajectories();
                twoPersonDrive.setCustomIntakeOutPosition(INTAKE_STACK_TOP_POSITION);
                visionPortal.stopStreaming();
            }
        }

        if (scanningStack2 && System.currentTimeMillis()-scanningStackStartTime>SCANNING_TIME + 1500) {
                scanningStack2 = false;
                runningTrajectory = true;
            trajectoryNumber = 2;
                drive.followTrajectorySequenceAsync(getStackPixels2);
        }

        if (!drive.isBusy() && trajectoryNumber == 3 && !atStack) {
            atStack = true;
            atStackStartTime = System.currentTimeMillis();
        }
        if (atStack && System.currentTimeMillis()-atStackStartTime<INTAKE_INTAKING_TIME) {
            if (!twoPersonDrive.intaking) {
                twoPersonDrive.intakingStartTime = System.currentTimeMillis();
            }
            twoPersonDrive.intaking = true;
            twoPersonDrive.updateIntake(1);
        }
        if (atStack && System.currentTimeMillis()-atStackStartTime>INTAKE_INTAKING_TIME) {
            twoPersonDrive.intaking = false;
            twoPersonDrive.updateIntake(0);
            atStack = false;
            if (trajectoryNumber == 3) {
                runningTrajectory = true;
                drive.followTrajectorySequenceAsync(scoreFirstStackPixels);
            }
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }

        if (!drive.isBusy() && trajectoryNumber == 4 && !atBackdrop) {
            atBackdrop = true;
            atBackdropStartTime = System.currentTimeMillis();
        }
        if (trajectoryNumber == 4 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME*2) {
            twoPersonDrive.outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
        }
        if (trajectoryNumber == 4 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME*3) {
            twoPersonDrive.leftOuttake.setPosition(LEFT_OUTTAKE_AVOID_POSITION);
            twoPersonDrive.rightOuttake.setPosition(RIGHT_OUTTAKE_AVOID_POSITION);
        }
        if (trajectoryNumber == 4 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME*4) {
            atBackdrop = false;
            runningTrajectory = true;
        }

        if (!drive.isBusy() && trajectoryNumber == 5 && !atBackdrop) {
            atBackdrop = true;
            atBackdropStartTime = System.currentTimeMillis();
        }
        if (trajectoryNumber == 5 && atBackdrop) {
            twoPersonDrive.leftOuttake.setPosition(LEFT_OUTTAKE_OUT_POSITION);
            twoPersonDrive.rightOuttake.setPosition(RIGHT_OUTTAKE_OUT_POSITION);
        }
        if (trajectoryNumber == 5 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME) {
            twoPersonDrive.innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);
        }
        if (trajectoryNumber == 5 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME*2) {
            atBackdrop = false;
            runningTrajectory = true;
            drive.followTrajectorySequenceAsync(park);
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }

        if (!drive.isBusy() && trajectoryNumber == 6 && !atBackdrop) {
            requestOpModeStop();
        }

        telemetry.addData("x error", stackRelocalization.getXErrorInches());
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