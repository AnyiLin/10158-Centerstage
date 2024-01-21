package org.firstinspires.ftc.teamcode.competition.autonomous;


import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_AUTO_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_STACK_MIDDLE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_STACK_TOP_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSE_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CLAW_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_FRONT_LENGTH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_POSITIONING;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_PRESET_HOLD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_RESET;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.competition.teleop.TwoPersonDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.VisionPortalTeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Right Inner Auto", group = "Autonomous")
public class RedRightInnerAuto extends OpMode {

    private TwoPersonDrive twoPersonDrive;

    private Timer pathTimer, opmodeTimer;

    private VisionPortalTeamPropPipeline teamPropPipeline;

    private VisionPortal visionPortal;

    private String navigation;

    private DistanceSensor leftDistanceSensor, rightDistanceSensor;

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

    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose, thirdCycleStackPose, thirdCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(63+72, 12+72, Math.PI);

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop, preBuildFirstCycleBackdrop, preBuildSecondCycleStack, preBuildSecondCycleBackdrop;
    private PathChain firstCycleToStack, firstCycleStackGrab, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleStackGrab, secondCycleScoreOnBackdrop;

    private int pathState;

    public void preBuildPaths() {
        firstCycleBackdropGoalPose = new Pose2d(redLeftBackdrop.getX(), redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
        secondCycleBackdropGoalPose = new Pose2d(redLeftBackdrop.getX(), redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);

        preBuildFirstCycleBackdrop = new Path(new BezierCurve(new Point(84, 79, Point.CARTESIAN), new Point(76.5, 106, Point.CARTESIAN), new Point(firstCycleBackdropGoalPose)));

        preBuildSecondCycleStack = new Path(new BezierCurve(new Point(firstCycleBackdropGoalPose), new Point(76.5, 106, Point.CARTESIAN), new Point(84, 79, Point.CARTESIAN)));

        preBuildSecondCycleBackdrop = new Path(new BezierCurve(new Point(84, 79, Point.CARTESIAN), new Point(76.5, 106, Point.CARTESIAN), new Point(secondCycleBackdropGoalPose)));
    }

    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose2d(redRightSideLeftSpikeMark.getX()-2.5, redRightSideLeftSpikeMark.getY()-1.5, Math.PI/2);
                initialBackdropGoalPose = new Pose2d(redLeftBackdrop.getX() + 3, redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(redRightSideMiddleSpikeMark.getX()+0.75, redRightSideMiddleSpikeMark.getY()-3, Math.PI/2);
                initialBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX() + 1, redMiddleBackdrop.getY()-ROBOT_BACK_LENGTH,Math.PI * 1.5);
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(redRightSideRightSpikeMark.getX()-2.5, redRightSideRightSpikeMark.getY(), Math.PI/2);
                initialBackdropGoalPose = new Pose2d(redRightBackdrop.getX() - 3, redRightBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                break;
        }
    }

    public void buildPaths() {
        Point scoreSpikeMarkMidPoint;
        double scoreSpikeMarkMidToSpikeDistance;
        switch (navigation) {
            default:
            case "left":
                scoreSpikeMarkMidPoint = new Point(122.5, 99, Point.CARTESIAN);
                scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
                scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() + Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() + Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
                break;
            case "middle":
                scoreSpikeMarkMidPoint = new Point(122, 84.5, Point.CARTESIAN);
                scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
                scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() + Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() + Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
                break;
            case "right":
                scoreSpikeMarkMidPoint = new Point(131.5, 82, Point.CARTESIAN);
                scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
                scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() + Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() - Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
                break;
        }
        scoreSpikeMark.setPathEndTimeout(2);

        switch (navigation) {
            default:
            case "left":
                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(scoreSpikeMark.getLastControlPoint().getX(), 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 109.5, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                break;
            case "middle":
                //initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(119, 103.5, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 106, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                initialScoreOnBackdrop = new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(initialBackdropGoalPose)));
                break;
            case "right":
                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(135, 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 106, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                break;
        }
        initialScoreOnBackdrop.setConstantHeadingInterpolation(Math.PI * 1.5);
        initialScoreOnBackdrop.setPathEndTimeout(2.5);

        switch (navigation) {
            default:
            case "left":
                firstCycleStackPose = new Pose2d(redInnerStack.getX() - 3.5, redInnerStack.getY() + ROBOT_FRONT_LENGTH + 1.75);
                secondCycleStackPose = new Pose2d(redInnerStack.getX() - 2.5, redInnerStack.getY() + ROBOT_FRONT_LENGTH + 1.75);
                break;
            case "middle":
                firstCycleStackPose = new Pose2d(redInnerStack.getX() - 3.5, redInnerStack.getY() + ROBOT_FRONT_LENGTH + 1.75);
                secondCycleStackPose = new Pose2d(redInnerStack.getX() - 1.3, redInnerStack.getY() + ROBOT_FRONT_LENGTH + 1.75);
                break;
            case "right":
                firstCycleStackPose = new Pose2d(redInnerStack.getX() - 2.75, redInnerStack.getY() + ROBOT_FRONT_LENGTH + 1.75);
                secondCycleStackPose = new Pose2d(redInnerStack.getX() - 2.5, redInnerStack.getY() + ROBOT_FRONT_LENGTH + 1.75);
                break;
        }

        firstCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(76.5, 106, Point.CARTESIAN), new Point(80, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .addPath(new BezierLine(new Point(84, 79, Point.CARTESIAN), new Point(firstCycleStackPose.getX(), 24, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                //.addParametricCallback(0, ()-> twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_TOP_POSITION))
                .build();

        firstCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX(), 24, Point.CARTESIAN), new Point(firstCycleStackPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .build();

        firstCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose), new Point(84, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .addPath(preBuildFirstCycleBackdrop)
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .setPathEndTimeout(2.5)
                .build();

        secondCycleToStack = follower.pathBuilder()
                .addPath(preBuildSecondCycleStack)
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .addPath(new BezierLine(new Point(84, 79, Point.CARTESIAN), new Point(secondCycleStackPose.getX(), 24, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                //.addParametricCallback(0, ()-> twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_MIDDLE_POSITION))
                .build();

        secondCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose.getX(), 24, Point.CARTESIAN), new Point(secondCycleStackPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .build();

        secondCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose), new Point(84, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .addPath(preBuildSecondCycleBackdrop)
                .setPathEndTimeout(2.5)
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // starts following the first path to score on the spike mark
                follower.followPath(scoreSpikeMark);
                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_OUT_POSITION+0.01);
                setPathState(11);
                break;
            case 11: // detects the path to progress away from the wall and sets tangent interpolation
                if (follower.getCurrentTValue() > 0.1) {
                    scoreSpikeMark.setReversed(false);
                    setPathState(12);
                }
                break;
            case 12: // detects for the end of the path and everything else to be in order and releases the pixel
                if (!follower.isBusy() && twoPersonDrive.intakeState == INTAKE_OUT) {
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_OPEN);
                    setPathState(13);
                }
                break;
            case 13: // moves mechanisms into position to score and pick up from stack as well as starts moving to score
                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_OUT);
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(14);
                }
                break;
            case 14: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    Follower.useHeading = false;
                    follower.holdPoint(new BezierPoint(new Point(follower.getPose())), Math.PI * 1.5);
                    twoPersonDrive.setOuttakeArmInterpolation(0.35);
                    setPathState(15);
                }
                break;
            case 15:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(16);
                }
                break;
            case 16: // detects for end of the path and outtake out and drops pixel
                if (pathTimer.getElapsedTime() > 500) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(20);
                }
                break;


            case 20: // starts the robot off on to the first stack once the pixels have been dropped
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    Follower.useHeading = true;
                    follower.followPath(firstCycleToStack);
                    setPathState(21);
                }
                break;
            case 21:
                if (follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.4 && follower.isBusy()) {
                    stackCorrection();
                }
                if (!follower.isBusy()) {
                    setPathState(22);
                }
                break;
            case 22:
                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_TOP_POSITION);
                setPathState(23);
                break;
            case 23:
                if (twoPersonDrive.intakeArmAtTargetPosition()) {
                    follower.followPath(firstCycleStackGrab);
                    setPathState(24);
                }
                break;
            case 24:
                if (follower.atParametricEnd()) {
                    follower.holdPoint(new BezierPoint(new Point(firstCycleStackPose)), Math.PI * 1.5);
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
                    setPathState(25);
                }
                break;
            case 25: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_POSITIONING);
                    follower.poseUpdater.resetOffset();
                    follower.followPath(firstCycleScoreOnBackdrop);
                    setPathState(26);
                }
                break;
            case 26:
                if (((follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.1) || !follower.isBusy()) && twoPersonDrive.transferState == TRANSFER_PRESET_HOLD) {
                    twoPersonDrive.liftPresetTargetPosition = 850;
                    twoPersonDrive.setTransferState(TRANSFER_OUT);
                    setPathState(27);
                }
                break;
            case 27: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.setOuttakeArmInterpolation(0.4);
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    Follower.useHeading = false;
                    follower.holdPoint(new BezierPoint(new Point(follower.getPose())), Math.PI * 1.5);
                    setPathState(28);
                }
                break;
            case 28:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(29);
                }
                break;
            case 29:
                if (pathTimer.getElapsedTime() > 300) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    setPathState(210);
                }
                break;
            case 210: // once the outer pixel has dropped, drop the inner one and fold up
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(30);
                }
                break;


            case 30: // once the inner pixel has dropped, start the robot off to the second pass on the first stack
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    Follower.useHeading = true;
                    follower.followPath(secondCycleToStack);
                    setPathState(31);
                }
                break;
            case 31:
                if (follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.4 && follower.isBusy()) {
                    stackCorrection();
                }
                if (!follower.isBusy()) {
                    setPathState(32);
                }
                break;
            case 32:
                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_MIDDLE_POSITION);
                setPathState(33);
                break;
            case 33:
                if (twoPersonDrive.intakeArmAtTargetPosition()) {
                    follower.followPath(secondCycleStackGrab);
                    setPathState(34);
                }
                break;
            case 34:
                if (follower.atParametricEnd()) {
                    follower.holdPoint(new BezierPoint(new Point(secondCycleStackPose)), Math.PI * 1.5);
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
                    setPathState(35);
                }
                break;
            case 35: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_POSITIONING);
                    follower.followPath(secondCycleScoreOnBackdrop);
                    setPathState(36);
                }
                break;
            case 36:
                if (((follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.1) || !follower.isBusy()) && twoPersonDrive.transferState == TRANSFER_PRESET_HOLD) {
                    twoPersonDrive.liftPresetTargetPosition = 850;
                    twoPersonDrive.setTransferState(TRANSFER_OUT);
                    setPathState(37);
                }
                break;
            case 37: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.setOuttakeArmInterpolation(0.4);
                    //twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    Follower.useHeading = false;
                    follower.holdPoint(new BezierPoint(new Point(follower.getPose())), Math.PI * 1.5);
                    setPathState(38);
                }
                break;
            case 38:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(39);
                }
                break;
            case 39:
                if (pathTimer.getElapsedTime() > 300) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    setPathState(310);
                }
                break;
            case 310: // once the outer pixel has dropped, drop the inner one and fold up
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    Follower.useHeading = true;
                    setPathState(40);
                }
                break;


            case 40: // move the intake in
                twoPersonDrive.moveIntake(INTAKE_IN);
                setPathState(41);
                break;
            case 41: // once the robot is nice and folded up, request stop
                if (twoPersonDrive.intakeState == INTAKE_IN && twoPersonDrive.intakeArmAtTargetPosition() && twoPersonDrive.outtakeState == OUTTAKE_IN && twoPersonDrive.outtakeArmAtTargetPosition()) {
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

    public void stackCorrection() {
        double error = leftDistanceSensor.getDistance(DistanceUnit.INCH)-rightDistanceSensor.getDistance(DistanceUnit.INCH);

        if (Math.abs(error) > 1) follower.poseUpdater.setXOffset(follower.poseUpdater.getXOffset() + 0.9 * MathFunctions.getSign(error));

        if (Math.abs(follower.poseUpdater.getXOffset()) > 2) follower.poseUpdater.setXOffset(2 * MathFunctions.getSign(follower.poseUpdater.getXOffset()));
    }

    @Override
    public void loop() {
        follower.update();
        twoPersonDrive.autonomousControlUpdate();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        double[] motorPowers = follower.motorPowers();
        for (int i = 0; i < motorPowers.length; i++) {
            telemetry.addData("motor " + i, motorPowers[i]);
        }
        twoPersonDrive.telemetry();
        //telemetry.update();
    }

    @Override
    public void init() {
        //PhotonCore.start(this.hardwareMap);

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        twoPersonDrive = new TwoPersonDrive(true);
        twoPersonDrive.hardwareMap = hardwareMap;
        twoPersonDrive.telemetry = telemetry;

        pathTimer = new Timer();
        opmodeTimer = new Timer();

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

        preBuildPaths();

        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);

        twoPersonDrive.innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_CLOSED);
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
        setPathState(10);
    }

    @Override
    public void stop() {
        super.stop();
    }
}

/**
 * 8==D
 */