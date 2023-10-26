package org.firstinspires.ftc.teamcode.competition.autonomous;

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
import org.firstinspires.ftc.teamcode.util.opencv.StackRelocalization;
import org.firstinspires.ftc.teamcode.util.opencv.TeamPropPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Experimental Red Right Inner Auto", group = "Autonomous")
public class ExperimentalRedRightInnerAuto extends OpMode {

    private TwoPersonDrive twoPersonDrive = new TwoPersonDrive(true);

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
            ROBOT_INTAKE_LENGTH = RobotConstants.ROBOT_INTAKE_LENGTH,
            INTAKE_STACK_TOP_POSITION = RobotConstants.INTAKE_STACK_TOP_POSITION,
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
            INTAKE_FULL_OUT_WAIT = RobotConstants.INTAKE_FULL_OUT_WAIT,
            INTAKE_BURST_TIME = RobotConstants.INTAKE_BURST_TIME;

    private final PIDFCoefficients
            LIFT_UP_VELOCITY_PIDF_COEFFICIENTS = RobotConstants.LIFT_UP_VELOCITY_PIDF_COEFFICIENTS,
            LIFT_UP_POSITION_PIDF_COEFFICIENTS = RobotConstants.LIFT_UP_POSITION_PIDF_COEFFICIENTS,
            LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS = RobotConstants.LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS,
            LIFT_DOWN_POSITION_PIDF_COEFFICIENTS = RobotConstants.LIFT_DOWN_POSITION_PIDF_COEFFICIENTS;

    private OpenCvCamera camera;

    private TeamPropPipeline teamPropPipeline = new TeamPropPipeline(0);

    private StackRelocalization stackRelocalization = new StackRelocalization();

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

    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose, firstStackPose, firstCycleFirstBackdropGoalPose, firstCycleSecondBackdropGoalPose, spikeMarkReturnPosition;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(12,-72+9, Math.toRadians(90));

    private SampleMecanumDrive drive;

    private TrajectorySequence scoreSpikeMark, getStackPixels, getStackPixels2, scoreFirstStackFirstPixel, scoreFirstStackSecondPixel, park;

    // these are auto specific timings and booleans
    private boolean scanningStack, scanningStack2, atBackdrop, runningTrajectory, atStack;

    private long scanningStackStartTime, initializationSlideResetStartTime, atBackdropStartTime, atStackStartTime;

    private final long BACKDROP_WAIT_TIME = 1000, SCORE_WAIT_TIME = 500, INTAKE_INTAKING_TIME = 2000;

    private int trajectoryNumber;

    public void setBackdropGoalPose() {
        switch (navigation) {
            case "left":
                spikeMarkGoalPose = new Pose2d(redRightSideLeftSpikeMark.getX()+(ROBOT_FRONT_LENGTH/Math.sqrt(2)), redRightSideLeftSpikeMark.getY()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), Math.toRadians(135));
                initialBackdropGoalPose = new Pose2d(redLeftBackdrop.getX()-ROBOT_BACK_LENGTH, 1.5-2.5+redLeftBackdrop.getY(), Math.toRadians(180));
                spikeMarkReturnPosition = new Pose2d(12, -36-11.5);
                firstCycleFirstBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5, -1.5-2.5+redMiddleBackdrop.getY(), Math.toRadians(180));
                firstCycleSecondBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.75, 1.5-1.5+redMiddleBackdrop.getY(), Math.toRadians(180));
                firstStackPose = new Pose2d(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH, redInnerStack.getY());
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(redRightSideMiddleSpikeMark.getX(), redRightSideMiddleSpikeMark.getY()-ROBOT_FRONT_LENGTH-1.5, Math.toRadians(90));
                initialBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH, -2+redMiddleBackdrop.getY(), Math.toRadians(180));
                spikeMarkReturnPosition = new Pose2d(12, -36-11.5);
                firstCycleFirstBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5, -2+redLeftBackdrop.getY(), Math.toRadians(180));
                firstCycleSecondBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.75, -2.75+redLeftBackdrop.getY(), Math.toRadians(180));
                firstStackPose = new Pose2d(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH-0.5, redInnerStack.getY()-0.25);
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(redRightSideRightSpikeMark.getX()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), redRightSideRightSpikeMark.getY()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), Math.toRadians(45));
                initialBackdropGoalPose = new Pose2d(redRightBackdrop.getX()-ROBOT_BACK_LENGTH, -1.5+redRightBackdrop.getY(), Math.toRadians(180));
                spikeMarkReturnPosition = new Pose2d(14, -36-11.5);
                firstCycleFirstBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5, -1.5+redMiddleBackdrop.getY(), Math.toRadians(180));
                firstCycleSecondBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.75, 1.5+redMiddleBackdrop.getY(), Math.toRadians(180));
                firstStackPose = new Pose2d(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH, redInnerStack.getY());
                break;
        }
    }

    public void buildTrajectories() {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        // this does the scoring on the spike mark at the start of auto
        scoreSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToConstantHeading(new Vector2d(12, -36-11.5))
                .lineToLinearHeading(spikeMarkGoalPose) // goes to the red spike mark location
                .resetConstraints()
                .lineToConstantHeading(new Vector2d(spikeMarkReturnPosition.getX(), spikeMarkReturnPosition.getY()))
                .lineToLinearHeading(new Pose2d(16, -56, Math.toRadians(90)))
                //.lineToLinearHeading(new Pose2d(40, -56, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0,()-> twoPersonDrive.startPreset(0, false))
                .lineToLinearHeading(new Pose2d(40, initialBackdropGoalPose.getY(), Math.toRadians(180)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(initialBackdropGoalPose)
                .resetConstraints()
                .build();

        getStackPixels = drive.trajectorySequenceBuilder(scoreSpikeMark.end())
                .lineToLinearHeading(new Pose2d(24, -12, Math.toRadians(180)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(firstStackPose.getX()+6, firstStackPose.getY()-0.0001, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1,()-> camera.setPipeline(stackRelocalization))
                .UNSTABLE_addTemporalMarkerOffset(-1,()-> camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT))
                .lineToLinearHeading(new Pose2d(firstStackPose.getX()+4, firstStackPose.getY(), Math.toRadians(180)))
                .resetConstraints()
                .build();
/*
        getStackPixels2 = drive.trajectorySequenceBuilder(getStackPixels.end())
                .UNSTABLE_addTemporalMarkerOffset(0,()-> twoPersonDrive.startPreset(0))
                .lineToLinearHeading(new Pose2d(40, firstCycleFirstBackdropGoalPose.getY(), Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(firstStackPose.getX(), firstStackPose.getY(), Math.toRadians(180)))
                .resetConstraints()
                .build();

        scoreFirstStackFirstPixel = drive.trajectorySequenceBuilder(getStackPixels2.end())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0,()-> twoPersonDrive.startPreset(0))
                .lineToLinearHeading(new Pose2d(40, firstCycleFirstBackdropGoalPose.getY(), Math.toRadians(180)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(firstCycleFirstBackdropGoalPose)
                .resetConstraints()
                .build();

        scoreFirstStackSecondPixel = drive.trajectorySequenceBuilder(scoreFirstStackFirstPixel.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(firstCycleSecondBackdropGoalPose)
                .resetConstraints()
                .build();

        park = drive.trajectorySequenceBuilder(scoreFirstStackSecondPixel.end())
                .lineToConstantHeading(new Vector2d(46, redMiddleBackdrop.getY()))
                .lineToConstantHeading(new Vector2d(46, -60))
                .build();
 */
    }

    public void stackAdjustTrajectories() {
        getStackPixels2 = drive.trajectorySequenceBuilder(getStackPixels.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(firstStackPose.getX(), firstStackPose.getY(), Math.toRadians(180)))
                .resetConstraints()
                .build();

        scoreFirstStackFirstPixel = drive.trajectorySequenceBuilder(getStackPixels2.end())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0,()-> twoPersonDrive.startPreset(0))
                .lineToLinearHeading(new Pose2d(40, firstCycleFirstBackdropGoalPose.getY(), Math.toRadians(180)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(firstCycleFirstBackdropGoalPose)
                .resetConstraints()
                .build();

        scoreFirstStackSecondPixel = drive.trajectorySequenceBuilder(scoreFirstStackFirstPixel.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(firstCycleSecondBackdropGoalPose)
                .resetConstraints()
                .build();

        park = drive.trajectorySequenceBuilder(scoreFirstStackSecondPixel.end())
                .lineToConstantHeading(new Vector2d(46, redMiddleBackdrop.getY()))
                .lineToConstantHeading(new Vector2d(46, -60))
                .build();
    }

    public void initialize() {
        twoPersonDrive.hardwareMap = hardwareMap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        camera.setPipeline(teamPropPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                onOpened();
            }
        });

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

        if (!drive.isBusy() && trajectoryNumber == 2 && !(scanningStack||scanningStack2)) {
            scanningStack = true;
            scanningStackStartTime = System.currentTimeMillis();
        }
        if (scanningStack && System.currentTimeMillis()-scanningStackStartTime>500) {
            firstStackPose = new Pose2d(firstStackPose.getX(), firstStackPose.getY()-stackRelocalization.getXErrorInches());
            stackAdjustTrajectories();
            twoPersonDrive.setCustomIntakeOutPosition(INTAKE_STACK_TOP_POSITION);
            camera.stopStreaming();
            scanningStack = false;
            scanningStack2 = true;
        }
        if (scanningStack2 && System.currentTimeMillis()-scanningStackStartTime>2000) {
            scanningStack2 = false;
            runningTrajectory = true;
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
                drive.followTrajectorySequenceAsync(scoreFirstStackFirstPixel);
            }
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }

        if (!drive.isBusy() && trajectoryNumber == 5 && !atStack) {
            atBackdrop = true;
            atBackdropStartTime = System.currentTimeMillis();
        }
        if (trajectoryNumber == 5 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > BACKDROP_WAIT_TIME) {
            twoPersonDrive.outerClaw.setPosition(OUTER_CLAW_OPEN_POSITION);
        }
        if (trajectoryNumber == 5 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > BACKDROP_WAIT_TIME+SCORE_WAIT_TIME) {
            twoPersonDrive.leftOuttake.setPosition(LEFT_OUTTAKE_AVOID_POSITION);
            twoPersonDrive.rightOuttake.setPosition(RIGHT_OUTTAKE_AVOID_POSITION);
            atBackdrop = false;
            runningTrajectory = true;
            drive.followTrajectorySequenceAsync(scoreFirstStackSecondPixel);
        }

        if (!drive.isBusy() && trajectoryNumber == 6 && !atBackdrop) {
            atBackdrop = true;
            atBackdropStartTime = System.currentTimeMillis();
        }
        if (trajectoryNumber == 6 && atBackdrop) {
            twoPersonDrive.leftOuttake.setPosition(LEFT_OUTTAKE_OUT_POSITION);
            twoPersonDrive.rightOuttake.setPosition(RIGHT_OUTTAKE_OUT_POSITION);
        }
        if (trajectoryNumber == 6 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > BACKDROP_WAIT_TIME) {
            twoPersonDrive.innerClaw.setPosition(INNER_CLAW_OPEN_POSITION);
        }
        if (trajectoryNumber == 6 && atBackdrop && System.currentTimeMillis()-atBackdropStartTime > BACKDROP_WAIT_TIME+SCORE_WAIT_TIME) {
            atBackdrop = false;
            runningTrajectory = true;
            drive.followTrajectorySequenceAsync(park);
            if (!twoPersonDrive.resetInMotion) twoPersonDrive.resetPreset();
        }

        if (!drive.isBusy() && trajectoryNumber == 7 && !atBackdrop) {
            requestOpModeStop();
        }

        telemetry.addData("x error", stackRelocalization.getXErrorInches());
        telemetry.addData("bounding boxes size", stackRelocalization.boundingBoxes.size());
        telemetry.addData("trajectory #", trajectoryNumber);
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
        //camera.stopStreaming();
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