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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.notcompetition.teleop.OldTwoPersonDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.VisionPortalStackRelocalization;
import org.firstinspires.ftc.teamcode.util.VisionPortalTeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Old Pedro Pathing Red Right Inner Auto", group = "Not Comp Autonomous")
public class OldPedroPathingRedRightInnerAuto extends OpMode {

    private OldTwoPersonDrive twoPersonDrive = new OldTwoPersonDrive(true);

    private VisionPortalTeamPropPipeline teamPropPipeline = new VisionPortalTeamPropPipeline(0);

    private VisionPortalStackRelocalization stackRelocalization = new VisionPortalStackRelocalization();

    private VisionPortal visionPortal;

    private String navigation;

    // all spike mark locations since I'm lazy
    private Pose redLeftSideLeftSpikeMark = new Pose(-47.5,-36);
    private Pose redLeftSideMiddleSpikeMark = new Pose(-36,-24.5);
    private Pose redLeftSideRightSpikeMark = new Pose(-24.5,-36);
    private Pose redRightSideLeftSpikeMark = new Pose(0.5,-36);
    private Pose redRightSideMiddleSpikeMark = new Pose(12,-24.5);
    private Pose redRightSideRightSpikeMark = new Pose(23.5,-36);
    private Pose blueLeftSideLeftSpikeMark = new Pose(23.5,36);
    private Pose blueLeftSideMiddleSpikeMark = new Pose(12,24.5);
    private Pose blueLeftSideRightSpikeMark = new Pose(0.5,36);
    private Pose blueRightSideLeftSpikeMark = new Pose(-24.5,36);
    private Pose blueRightSideMiddleSpikeMark = new Pose(-36,24.5);
    private Pose blueRightSideRightSpikeMark = new Pose(-47.5,36);

    // backdrop april tag locations
    private Pose blueLeftBackdrop = new Pose(60.75, 72-22.5-6.625);
    private Pose blueMiddleBackdrop = new Pose(60.75, 72-22.5-12.75);
    private Pose blueRightBackdrop = new Pose(60.75, 72-22.5-18.75);
    private Pose redLeftBackdrop = new Pose(60.75, -72+22.5+18.75);
    private Pose redMiddleBackdrop = new Pose(60.75, -72+22.5+12.75);
    private Pose redRightBackdrop = new Pose(60.75, -72+22.5+6.625);

    // white pixel stack locations
    private Pose redOuterStack = new Pose(-72, -72+36);
    private Pose redMiddleStack = new Pose(-72, -72+48);
    private Pose redInnerStack = new Pose(-72, -72+60);
    private Pose blueInnerStack = new Pose(-72, 72-60);
    private Pose blueMiddleStack = new Pose(-72, 72-48);
    private Pose blueOuterStack = new Pose(-72, 72-36);

    private Pose spikeMarkGoalPose, initialBackdropGoalPose, stackPose, firstCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(12,-72+9, Math.toRadians(90));

    private Follower follower;

    private Path scoreOnSpikeMark, goBackFromSpikeMark, scoreOnBackdrop, adjustStack, actuallyGetStackPixels;

    private PathChain getStackPixels, scoreStackPixels;

    // these are auto specific timings and booleans
    private boolean scanningStack, actuallyGoingToStack, atBackdrop, atStack, hasActivatedStackRelocalization, goToStackTimeout;

    private long scanningStackStartTime, scanningTime, initializationSlideResetStartTime, atBackdropStartTime, atStackStartTime;

    private final long BACKDROP_WAIT_TIME = 0*1000, SCANNING_TIME = 1000, SCORE_WAIT_TIME = 500, INTAKE_INTAKING_TIME = 2000;

    private int pathNumber;

    public void setBackdropGoalPose() {
        switch (navigation) {
            case "left":
                spikeMarkGoalPose = new Pose(redRightSideLeftSpikeMark.getX()+(ROBOT_FRONT_LENGTH/Math.sqrt(2))-1.5, redRightSideLeftSpikeMark.getY());
                initialBackdropGoalPose = new Pose(redLeftBackdrop.getX()-ROBOT_BACK_LENGTH -0.5, 2.5-2.5+redLeftBackdrop.getY() -0.5, Math.toRadians(180));
                firstCycleBackdropGoalPose = new Pose(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5 -0.5, -1.5-2.5+redMiddleBackdrop.getY(), Math.toRadians(180));
                stackPose = new Pose(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH+1.5, redInnerStack.getY()-3);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose(redRightSideMiddleSpikeMark.getX(), redRightSideMiddleSpikeMark.getY()-ROBOT_FRONT_LENGTH-1.25, Math.toRadians(90));
                initialBackdropGoalPose = new Pose(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH -0.5, -2+redMiddleBackdrop.getY() -0.5, Math.toRadians(180));
                firstCycleBackdropGoalPose = new Pose(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5 -0.5, -2+redLeftBackdrop.getY(), Math.toRadians(180));
                stackPose = new Pose(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH+0.75, redInnerStack.getY()-2);
                break;
            case "right":
                spikeMarkGoalPose = new Pose(redRightSideRightSpikeMark.getX()-(ROBOT_FRONT_LENGTH/Math.sqrt(2))+1.5, redRightSideRightSpikeMark.getY());
                initialBackdropGoalPose = new Pose(redRightBackdrop.getX()-ROBOT_BACK_LENGTH -0.5, -1.5+redRightBackdrop.getY() -0.25, Math.toRadians(180));
                firstCycleBackdropGoalPose = new Pose(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5 -0.5, -1.5+redMiddleBackdrop.getY(), Math.toRadians(180));
                stackPose = new Pose(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH+0.75, redInnerStack.getY()-2);
                break;
        }
    }

    public void buildPaths() {
        scoreOnSpikeMark = new Path(new BezierCurve(new Point(startPose), new Point(startPose.getX(), spikeMarkGoalPose.getY()-6, Point.CARTESIAN), new Point(spikeMarkGoalPose)));


        goBackFromSpikeMark = new Path(new BezierLine(new Point(spikeMarkGoalPose), new Point(startPose.getX()+5, spikeMarkGoalPose.getY()-6-5, Point.CARTESIAN)));
        goBackFromSpikeMark.setReversed(true);

        scoreOnBackdrop = new Path(new BezierCurve(new Point(startPose.getX()+3, -48, Point.CARTESIAN), new Point(33, -58, Point.CARTESIAN), new Point(31, initialBackdropGoalPose.getY(), Point.CARTESIAN), new Point(initialBackdropGoalPose)));
        scoreOnBackdrop.setConstantHeadingInterpolation(Math.PI);
        scoreOnBackdrop.setPathEndTimeoutConstraint(2);

        getStackPixels = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(34, initialBackdropGoalPose.getY(), Point.CARTESIAN), new Point(42, -12, Point.CARTESIAN), new Point(12, -12, Point.CARTESIAN)))
                .addParametricCallback(0, ()-> visionPortal.resumeStreaming())
                .addParametricCallback(0, ()-> visionPortal.setProcessorEnabled(teamPropPipeline, false))
                .addParametricCallback(0, ()-> visionPortal.setProcessorEnabled(stackRelocalization, true))
                .addPath(new BezierLine(new Point(12, -12, Point.CARTESIAN), new Point(stackPose.getX()+6, stackPose.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI)
                .build();
    }

    public void adjustStack() {
        adjustStack = new Path(new BezierPoint(new Point(stackPose.getX()+6, stackPose.getY(), Point.CARTESIAN)));
        adjustStack.setConstantHeadingInterpolation(Math.PI);
    }

    public void stackAdjustPaths() {
        actuallyGetStackPixels = new Path(new BezierLine(new Point(-48, -12, Point.CARTESIAN), new Point(stackPose)));
        actuallyGetStackPixels.setConstantHeadingInterpolation(Math.PI);
        actuallyGetStackPixels.setPathEndHeadingConstraint(0.01);

        scoreStackPixels = follower.pathBuilder()
                .addPath(new BezierLine(new Point(stackPose), new Point(-36, -12, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(new Point(-36, -12, Point.CARTESIAN), new Point(12, -12, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.75, ()-> twoPersonDrive.startPreset(50))
                .addPath(new BezierCurve(new Point(12, -12, Point.CARTESIAN), new Point(42, -12, Point.CARTESIAN), new Point(34, firstCycleBackdropGoalPose.getY(), Point.CARTESIAN), new Point(firstCycleBackdropGoalPose)))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        /*
        getStackPixels2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose(firstStackPose.getX(), firstStackPose.getY(), Math.toRadians(180)))
                .resetConstraints()
                .build();

        scoreFirstStackPixels = drive.trajectorySequenceBuilder(getStackPixels2.end())
                .lineToLinearHeading(new Pose(-36, -12, Math.toRadians(180)))
                .setReversed(true)
                .splineToSplineHeading(new Pose(27, -12, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()-> twoPersonDrive.startPreset(50))
                .splineToLinearHeading(new Pose(40, firstCycleBackdropGoalPose.getY(), Math.toRadians(180)), Math.toRadians(180))
                .setReversed(false)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(firstCycleBackdropGoalPose)
                .resetConstraints()
                .build();

        park = drive.trajectorySequenceBuilder(scoreFirstStackPixels.end())
                .lineToConstantHeading(new Vector2d(46, redMiddleBackdrop.getY()))
                .build();
         */
    }

    public void initialize() {
        twoPersonDrive.hardwareMap = hardwareMap;

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

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
        follower.update();
        twoPersonDrive.updateFrameTime();
        twoPersonDrive.asyncTimers();
        if (!twoPersonDrive.adjustingLiftZero) twoPersonDrive.updateLiftMotors();
        if (twoPersonDrive.presetInMotion) twoPersonDrive.detectPresetEnd();

        // After push pixel
        if (pathNumber == 0 && follower.atParametricEnd()) {
            follower.followPath(goBackFromSpikeMark);
            pathNumber = 1;
        }

        // go to backdrop
        if (pathNumber == 1 && follower.getCurrentTValue() > 0.7) {
            follower.followPath(scoreOnBackdrop);
            pathNumber = 2;
        }

        // score at backdrop
        if (pathNumber == 2 && (!follower.isBusy() || (follower.getCurrentTValue() > 0.9 && follower.getVelocityMagnitude() < 3)) && !atBackdrop) {
            atBackdrop = true;
            atBackdropStartTime = System.currentTimeMillis();
        }
        if (pathNumber == 2 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > BACKDROP_WAIT_TIME) {
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }
        if (pathNumber == 2 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > BACKDROP_WAIT_TIME+SCORE_WAIT_TIME) {
            // after scoring on the backdrop, do something else
            pathNumber = 3;
            follower.followPath(getStackPixels);
            atBackdrop = false;
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }

        // start readjustments
        if (pathNumber == 3 && !follower.isBusy() && !(scanningStack || actuallyGoingToStack || goToStackTimeout)) {
            scanningTime = System.currentTimeMillis()-1000;
            scanningStackStartTime = System.currentTimeMillis();
            scanningStack = true;
        }

        // readjustments
        if (scanningStack) {
            if (System.currentTimeMillis()-scanningTime>700) {
                stackPose = new Pose(stackPose.getX(), follower.getPose().getY()+stackRelocalization.getXErrorInches());
                adjustStack();
                follower.followPath(adjustStack);
                scanningTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis()-scanningStackStartTime>SCANNING_TIME) {
                scanningStack = false;
                actuallyGoingToStack = true;
                stackAdjustPaths();
                twoPersonDrive.setCustomIntakeOutPosition(INTAKE_STACK_TOP_POSITION);
                visionPortal.stopStreaming();
            }
        }

        if (actuallyGoingToStack && System.currentTimeMillis()-scanningStackStartTime>SCANNING_TIME + 1500) {
            actuallyGoingToStack = false;
            goToStackTimeout = true;
            pathNumber = 4;
            follower.followPath(actuallyGetStackPixels);
        }
        if (goToStackTimeout && System.currentTimeMillis()-scanningStackStartTime>SCANNING_TIME + 3500) {
            goToStackTimeout = false;
            follower.breakFollowing();
        }

        if (pathNumber == 4 && !(goToStackTimeout || atStack)) {
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
            twoPersonDrive.startPreset(0);
        }
        if (atStack && System.currentTimeMillis()-atStackStartTime>INTAKE_INTAKING_TIME+1000) {
            atStack = false;
            pathNumber = 5;
            follower.followPath(scoreStackPixels);
        }

        if ((!follower.isBusy() || (follower.getCurrentTValue() > 0.9 && follower.getVelocityMagnitude() < 3)) && pathNumber == 5 && !atBackdrop) {
            atBackdrop = true;
            atBackdropStartTime = System.currentTimeMillis();
        }
        if (pathNumber == 5 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME+200) {
            twoPersonDrive.outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
        }
        if (pathNumber == 5 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME*2+200) {
            twoPersonDrive.leftOuttake.setPosition(LEFT_OUTTAKE_AVOID_POSITION);
            twoPersonDrive.rightOuttake.setPosition(RIGHT_OUTTAKE_AVOID_POSITION);
        }
        if (pathNumber == 5 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME*3) {
            twoPersonDrive.leftOuttake.setPosition(LEFT_OUTTAKE_OUT_POSITION);
            twoPersonDrive.rightOuttake.setPosition(RIGHT_OUTTAKE_OUT_POSITION);
        }
        if (pathNumber == 5 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME*4) {
            twoPersonDrive.innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);
            twoPersonDrive.resetPreset();
            atBackdrop = false;
            pathNumber = 6;
        }

        if (!follower.isBusy() && pathNumber == 6) {
            requestOpModeStop();
        }

        /*

        if (!drive.isBusy() && pathNumber == 1 && !atBackdrop) {
            atBackdrop = true;
            atBackdropStartTime = System.currentTimeMillis();
        }
        if (pathNumber == 1 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > BACKDROP_WAIT_TIME) {
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }
        if (pathNumber == 1 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > BACKDROP_WAIT_TIME+SCORE_WAIT_TIME) {
            // after scoring on the backdrop, do something else
            runningPath = true;
            drive.followTrajectorySequenceAsync(getStackPixels);
            atBackdrop = false;
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }

        if (scanningStack) {
            if (System.currentTimeMillis()-scanningTime>700) {
                firstStackPose = new Pose(firstStackPose.getX(), drive.getPoseEstimate().getY()+stackRelocalization.getXErrorInches());
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
                runningPath = true;
            pathNumber = 2;
                drive.followTrajectorySequenceAsync(getStackPixels2);
        }

        if (!drive.isBusy() && pathNumber == 3 && !atStack) {
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
            if (pathNumber == 3) {
                runningPath = true;
                drive.followTrajectorySequenceAsync(scoreFirstStackPixels);
            }
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }

        if (!drive.isBusy() && pathNumber == 4 && !atBackdrop) {
            atBackdrop = true;
            atBackdropStartTime = System.currentTimeMillis();
        }
        if (pathNumber == 4 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME+200) {
            twoPersonDrive.outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
        }
        if (pathNumber == 4 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME*2+200) {
            twoPersonDrive.leftOuttake.setPosition(LEFT_OUTTAKE_AVOID_POSITION);
            twoPersonDrive.rightOuttake.setPosition(RIGHT_OUTTAKE_AVOID_POSITION);
        }
        if (pathNumber == 4 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME*3) {
            atBackdrop = false;
            runningPath = true;
        }

        if (!drive.isBusy() && pathNumber == 5 && !atBackdrop) {
            atBackdrop = true;
            atBackdropStartTime = System.currentTimeMillis();
        }
        if (pathNumber == 5 && atBackdrop) {
            twoPersonDrive.leftOuttake.setPosition(LEFT_OUTTAKE_OUT_POSITION);
            twoPersonDrive.rightOuttake.setPosition(RIGHT_OUTTAKE_OUT_POSITION);
        }
        if (pathNumber == 5 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME) {
            twoPersonDrive.innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);
        }
        if (pathNumber == 5 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > SCORE_WAIT_TIME*2) {
            atBackdrop = false;
            runningPath = true;
            drive.followTrajectorySequenceAsync(park);
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }

        if (!drive.isBusy() && pathNumber == 6 && !atBackdrop) {
            requestOpModeStop();
        }
         */

        telemetry.addData("x error", stackRelocalization.getXErrorInches());
        telemetry.addData("path #", pathNumber);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
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
        buildPaths();

    }

    @Override
    public void start() {
        super.start();
        visionPortal.stopStreaming();
        follower.followPath(scoreOnSpikeMark);
        twoPersonDrive.startPreset(0, false);
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