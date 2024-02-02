package org.firstinspires.ftc.teamcode.competition.autonomous;


import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_AUTO_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_STACK_MIDDLE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_STACK_TOP_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSE_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_TRANSFER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_CYCLE_SCORE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_YELLOW_SCORE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CLAW_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_FRONT_LENGTH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_POSITIONING;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_PRESET_HOLD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_RESET;
import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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

@Autonomous(name = "Blue Left Inner Auto", group = "Autonomous")
public class BlueLeftInnerAuto extends OpMode {

    private TwoPersonDrive twoPersonDrive;

    private Timer pathTimer, opmodeTimer, scanTimer;

    private VisionPortalTeamPropPipeline teamPropPipeline;

    private VisionPortal visionPortal;

    private String navigation;

    private DistanceSensor leftDistanceSensor, rightDistanceSensor;

    private RevColorSensorV3 colorSensor;

    private boolean distanceSensorDisconnected, colorSensorDisconnected;

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

    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(144-(63+72), 12+72, 0);

    // TODO: dont forget to adjust this too
    private Point abortPoint =  new Point(144-83.5, 120, Point.CARTESIAN);

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private PathChain firstCycleToStack, firstCycleStackGrab, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleStackGrab, secondCycleScoreOnBackdrop;

    private int pathState;

    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose2d(blueLeftSideLeftSpikeMark.getX() + 2.5, blueLeftSideLeftSpikeMark.getY()-0.75, Math.PI/2);
                initialBackdropGoalPose = new Pose2d(blueLeftBackdrop.getX(), blueLeftBackdrop.getY()-ROBOT_BACK_LENGTH-0.75, Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX(), blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-0.25, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX(), blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-0.25, Math.PI * 1.5);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(blueLeftSideMiddleSpikeMark.getX()+1.5, blueLeftSideMiddleSpikeMark.getY()+3, Math.PI/2);
                initialBackdropGoalPose = new Pose2d(blueMiddleBackdrop.getX() + 0.5, blueMiddleBackdrop.getY()-ROBOT_BACK_LENGTH-1,Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX()+1.5, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-0.5, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX()+1.5, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-0.25, Math.PI * 1.5);
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(blueLeftSideRightSpikeMark.getX() + 2.5, blueLeftSideRightSpikeMark.getY()-1.5, Math.PI/2);
                initialBackdropGoalPose = new Pose2d(blueRightBackdrop.getX()+1.5,blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-1, Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX(), blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-0.25, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX(), blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-0.25, Math.PI * 1.5);
                break;
        }
    }

    public void buildPaths() {
        Point scoreSpikeMarkMidPoint;
        double scoreSpikeMarkMidToSpikeDistance;
        switch (navigation) {
            default:
            case "left":
                scoreSpikeMarkMidPoint = new Point(144-131.5, 82, Point.CARTESIAN);
                scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
                scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() - Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() - Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
                break;
            case "middle":
                scoreSpikeMarkMidPoint = new Point(144-129.5, 106, Point.CARTESIAN);
                scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
                scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() - Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() + Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
                break;
            case "right":
                scoreSpikeMarkMidPoint = new Point(144-122.5, 99, Point.CARTESIAN);
                scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
                scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() - Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() + Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
                break;
        }
        scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());
        scoreSpikeMark.setPathEndTimeout(3);

        switch (navigation) {
            default:
            case "left":
                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(scoreSpikeMark.getLastControlPoint().getX(), 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 109.5, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                break;
            case "middle":
                initialScoreOnBackdrop = new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(initialBackdropGoalPose)));
                break;
            case "right":
                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(144-135, 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 106, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                break;
        }
        initialScoreOnBackdrop.setConstantHeadingInterpolation(Math.PI * 1.5);
        initialScoreOnBackdrop.setPathEndTimeout(2.5);

        switch (navigation) {
            default:
            case "left":
                firstCycleStackPose = new Pose2d(blueInnerStack.getX()-0.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH+2.5, Math.PI * 1.5 + Math.toRadians(2));
                secondCycleStackPose = new Pose2d(blueInnerStack.getX()-0.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH+2, Math.PI * 1.5 + Math.toRadians(0));
                break;
            case "middle":
                firstCycleStackPose = new Pose2d(blueInnerStack.getX()+0.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH+2, Math.PI * 1.5 + Math.toRadians(2));
                secondCycleStackPose = new Pose2d(blueInnerStack.getX()+0.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH+2, Math.PI * 1.5 + Math.toRadians(0));
                break;
            case "right":
                firstCycleStackPose = new Pose2d(blueInnerStack.getX()-0.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH+2, Math.PI * 1.5 + Math.toRadians(2));
                secondCycleStackPose = new Pose2d(blueInnerStack.getX()-0.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH+2, Math.PI * 1.5 + Math.toRadians(1));
                break;
        }

        firstCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(firstCycleStackPose.getX()+1, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX()+1, 79, Point.CARTESIAN), new Point(firstCycleStackPose.getX(), 21, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .setPathEndTimeout(0)
                .build();

        firstCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX()+0.0001, 32, Point.CARTESIAN), new Point(firstCycleStackPose)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .build();

        firstCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose), new Point(firstCycleStackPose.getX()+1, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .addPath(new BezierCurve(new Point(firstCycleStackPose.getX()+1, 79, Point.CARTESIAN), new Point(144-76.5, 106, Point.CARTESIAN), new Point(firstCycleBackdropGoalPose)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .setPathEndTimeout(2.5)
                .build();

        secondCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstCycleBackdropGoalPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(secondCycleStackPose.getX()+1, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .addPath(new BezierLine(new Point(secondCycleStackPose.getX()+1, 79, Point.CARTESIAN), new Point(secondCycleStackPose.getX(), 21, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .setPathEndTimeout(0)
                .build();

        secondCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose.getX()+0.0001, 32, Point.CARTESIAN), new Point(secondCycleStackPose)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .build();

        secondCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose), new Point(secondCycleStackPose.getX()+1, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .addPath(new BezierCurve(new Point(secondCycleStackPose.getX()+1, 79, Point.CARTESIAN), new Point(144-76.5, 106, Point.CARTESIAN), new Point(secondCycleBackdropGoalPose)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .setPathEndTimeout(2.5)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // starts following the first path to score on the spike mark
                follower.followPath(scoreSpikeMark);
                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_OUT_POSITION + 0.01);
                setPathState(11);
                break;
            case 11: // detects the path to progress away from the wall and sets tangent interpolation
                if (follower.getCurrentTValue() > 0.1) {
                    //scoreSpikeMark.setReversed(false);
                    scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading() - 0.1 * MathFunctions.getTurnDirection(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()) * MathFunctions.getSmallestAngleDifference(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()), scoreSpikeMark.getEndTangent().getTheta());
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
                    twoPersonDrive.outtakeWristOffset = -15;
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(15);
                }
                break;
            case 15: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    Follower.useHeading = false;
                    follower.holdPoint(new BezierPoint(new Point(follower.getPose())), Math.PI * 1.5);
                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_YELLOW_SCORE_POSITION);
                    setPathState(16);
                }
                break;
            case 16:
                colorSensorDisconnected = colorSensorDisconnected();
                if (colorSensorDisconnected) {
                    setPathState(17);
                    break;
                }
                backdropCorrection();
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(17);
                }
                break;
            case 17:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(18);
                }
                break;
            case 18: // detects for end of the path and outtake out and drops pixel
                if (pathTimer.getElapsedTime() > 500) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTime() > 500) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(20);
                }
                break;


            case 20: // starts the robot off on to the first stack once the pixels have been dropped
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    distanceSensorDisconnected = leftDistanceSensorDisconnected() || rightDistanceSensorDisconnected();
                    Follower.useHeading = true;
                    follower.poseUpdater.resetOffset();
                    if (distanceSensorDisconnected) {
                        setPathState(50);
                        break;
                    }
                    follower.followPath(firstCycleToStack);
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(firstCycleToStack.getPath(1).getLastControlPoint().getX(), firstCycleToStack.getPath(1).getLastControlPoint().getY() + 1, Point.CARTESIAN)), firstCycleStackPose.getHeading());
                    setPathState(22);
                }
                break;
            case 22:
                distanceSensorDisconnected = leftDistanceSensorDisconnected() || rightDistanceSensorDisconnected();
                if (distanceSensorDisconnected) {
                    setPathState(50);
                    break;
                }
                stackCorrection();
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(23);
                }
                break;
            case 23:
                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_TOP_POSITION);
                setPathState(24);
                break;
            case 24:
                if (twoPersonDrive.intakeArmAtTargetPosition()) {
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(firstCycleStackGrab);
                    setPathState(26);
                }
                break;
            case 26:
                if (follower.getCurrentTValue() > 0.92) {//!follower.isBusy()) {
                    //Follower.useHeading = false;
                    //follower.holdPoint(new BezierPoint(new Point(firstCycleStackPose)), Math.PI * 1.5);
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
                    setPathState(27);
                }
                if (pathTimer.getElapsedTime() > 1500) {
                    setPathState(27);
                }
                break;
            case 27: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_POSITIONING);
                    follower.poseUpdater.resetOffset();
                    Follower.useHeading = true;
                    follower.followPath(firstCycleScoreOnBackdrop);
                    setPathState(28);
                }
                break;
            case 28:
                if (((follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.1) || !follower.isBusy()) && twoPersonDrive.transferState == TRANSFER_PRESET_HOLD) {
                    twoPersonDrive.setTransferState(TRANSFER_OUT);
                    setPathState(29);
                }
                break;
            case 29: // detects for end of the path and outtake out and drops pixel
                if (follower.atParametricEnd() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.setLiftTargetPosition(850);
                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_CYCLE_SCORE_POSITION, 100);
                }
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    //Follower.useHeading = false;
                    follower.holdPoint(new BezierPoint(new Point(follower.getPose())), Math.PI * 1.5);
                    setPathState(210);
                }
                break;
            case 210:
                colorSensorDisconnected = colorSensorDisconnected();
                if (colorSensorDisconnected) {
                    setPathState(211);
                    break;
                }
                backdropCorrection();
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(211);
                }
                break;
            case 211:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(212);
                }
                break;
            case 212:
                if (pathTimer.getElapsedTime() > 300) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    setPathState(213);
                }
                break;
            case 213:
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_OUT_POSITION, 300);
                    setPathState(214);
                }
                break;
            case 214:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(215);
                }
                break;
            case 215:
                if (pathTimer.getElapsedTime() > 300) {
                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_CYCLE_SCORE_POSITION, 100);
                    setPathState(216);
                }
                break;
            case 216:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(217);
                }
                break;
            case 217: // once the outer pixel has dropped, drop the inner one and fold up
                if (pathTimer.getElapsedTime() > 2 * OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(30);
                }
                break;


            case 30: // once the inner pixel has dropped, start the robot off to the second pass on the first stack
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    distanceSensorDisconnected = leftDistanceSensorDisconnected() || rightDistanceSensorDisconnected();
                    Follower.useHeading = true;
                    follower.poseUpdater.resetOffset();
                    if (distanceSensorDisconnected) {
                        setPathState(50);
                        break;
                    }
                    follower.followPath(secondCycleToStack);
                    setPathState(31);
                }
                break;
            case 31:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(secondCycleToStack.getPath(1).getLastControlPoint().getX(), secondCycleToStack.getPath(1).getLastControlPoint().getY() + 1, Point.CARTESIAN)), secondCycleStackPose.getHeading());
                    setPathState(32);
                }
                break;
            case 32:
                distanceSensorDisconnected = leftDistanceSensorDisconnected() || rightDistanceSensorDisconnected();
                if (distanceSensorDisconnected) {
                    setPathState(50);
                    break;
                }
                stackCorrection();
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(33);
                }
                break;
            case 33:
                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_MIDDLE_POSITION);
                setPathState(34);
                break;
            case 34:
                if (twoPersonDrive.intakeArmAtTargetPosition()) {
                    setPathState(35);
                }
                break;
            case 35:
                if (pathTimer.getElapsedTime() > 300) {
                    follower.followPath(secondCycleStackGrab);
                    setPathState(36);
                }
                break;
            case 36:
                if (follower.getCurrentTValue() > 0.92) {//!follower.isBusy()) {
                    //Follower.useHeading = false;
                    //follower.holdPoint(new BezierPoint(new Point(secondCycleStackPose)), Math.PI * 1.5);
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
                    setPathState(37);
                }
                if (pathTimer.getElapsedTime() > 1500) {
                    setPathState(37);
                }
                break;
            case 37: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_POSITIONING);
                    Follower.useHeading = true;
                    follower.poseUpdater.resetOffset();
                    follower.followPath(secondCycleScoreOnBackdrop);
                    setPathState(38);
                }
                break;
            case 38:
                if (((follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.1) || !follower.isBusy()) && twoPersonDrive.transferState == TRANSFER_PRESET_HOLD) {
                    twoPersonDrive.setTransferState(TRANSFER_OUT);
                    setPathState(39);
                }
                break;
            case 39: // detects for end of the path and outtake out and drops pixel
                if (follower.atParametricEnd() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.setLiftTargetPosition(850);
                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_CYCLE_SCORE_POSITION, 100);
                }
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    //twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    //Follower.useHeading = false;
                    follower.holdPoint(new BezierPoint(new Point(follower.getPose())), Math.PI * 1.5);
                    setPathState(310);
                }
                break;
            case 310:
                colorSensorDisconnected = colorSensorDisconnected();
                if (colorSensorDisconnected) {
                    setPathState(311);
                    break;
                }
                backdropCorrection();
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(311);
                }
                break;
            case 311:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(312);
                }
                break;
            case 312:
                if (pathTimer.getElapsedTime() > 300) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    setPathState(313);
                }
                break;
            case 313:
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_OUT_POSITION, 300);
                    setPathState(314);
                }
                break;
            case 314:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(315);
                }
                break;
            case 315:
                if (pathTimer.getElapsedTime() > 300) {
                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_CYCLE_SCORE_POSITION, 100);
                    setPathState(316);
                }
                break;
            case 316:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(317);
                }
                break;
            case 317: // once the outer pixel has dropped, drop the inner one and fold up
                if (pathTimer.getElapsedTime() > 2 * OUTTAKE_CLAW_DROP_TIME) {
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

            case 50:
                twoPersonDrive.moveIntake(INTAKE_IN);
                setPathState(51);
                break;
            case 51:
                if (twoPersonDrive.intakeState == INTAKE_IN && twoPersonDrive.intakeArmAtTargetPosition() && twoPersonDrive.outtakeState == OUTTAKE_IN && twoPersonDrive.outtakeArmAtTargetPosition() && twoPersonDrive.liftEncoder.getCurrentPosition() < LIFT_TRANSFER_UPPER_LIMIT) {
                    follower.poseUpdater.resetOffset();
                    PathChain abort = follower.pathBuilder()
                            .addPath(new BezierLine(new Point(follower.poseUpdater.getPose()), abortPoint))
                            .setConstantHeadingInterpolation(Math.PI * 1.5)
                            .build();
                    follower.followPath(abort);
                    setPathState(52);
                }
                break;
            case 52:
                if (!follower.isBusy()) {
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

    public boolean stackCorrection() {
        double error = leftDistanceSensor.getDistance(DistanceUnit.INCH) - rightDistanceSensor.getDistance(DistanceUnit.INCH);
        if (!(leftDistanceSensor.getDistance(DistanceUnit.MM) == 65535 || rightDistanceSensor.getDistance(DistanceUnit.MM) == 65535)) {

            if (Math.abs(error) > 0.85) {
                follower.poseUpdater.setXOffset(follower.poseUpdater.getXOffset() + twoPersonDrive.deltaTimeSeconds * 6 * MathFunctions.getSign(error));
            } else {
                follower.poseUpdater.setXOffset(follower.getTranslationalError().getXComponent());
            }
        /*
        if (Math.abs(error) > 0.85) {
            follower.poseUpdater.setXOffset(follower.poseUpdater.getXOffset() + twoPersonDrive.deltaTimeSeconds * 3 * error);
        } else {
            follower.poseUpdater.setXOffset(follower.getTranslationalError().getXComponent());
        }
         */

            if (Math.abs(follower.poseUpdater.getXOffset()) > 6)
                follower.poseUpdater.setXOffset(6 * MathFunctions.getSign(follower.poseUpdater.getXOffset()));
        } else {
            return false;
        }
        telemetry.addData("error", error);

        return true;
    }

    public boolean leftDistanceSensorDisconnected() {
        return leftDistanceSensor.getDistance(DistanceUnit.MM) == 65535;
    }

    public boolean rightDistanceSensorDisconnected() {
        return rightDistanceSensor.getDistance(DistanceUnit.MM) == 65535;
    }

    public boolean backdropCorrection() {
        double rawLightValue = colorSensor.getRawLightDetected();

        if (!(rawLightValue == 0)) {
            // too close
            if (rawLightValue > 150)
                follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() + twoPersonDrive.deltaTimeSeconds * 4);

            // too far
            if (rawLightValue < 133)
                follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() - twoPersonDrive.deltaTimeSeconds * 6);

            if (Math.abs(follower.poseUpdater.getYOffset()) > 1.5)
                follower.poseUpdater.setYOffset(1.5 * MathFunctions.getSign(follower.poseUpdater.getYOffset()));
        } else {
            return false;
        }
        telemetry.addData("raw light value", rawLightValue);

        return true;
    }

    public boolean colorSensorDisconnected() {
        return colorSensor.getRawLightDetected() == 0;
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

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        twoPersonDrive = new TwoPersonDrive(true);
        twoPersonDrive.hardwareMap = hardwareMap;
        twoPersonDrive.telemetry = telemetry;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        teamPropPipeline = new VisionPortalTeamPropPipeline(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessors(teamPropPipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        twoPersonDrive.initialize();
        twoPersonDrive.setIntakeArmPosition(INTAKE_ARM_IN_POSITION+0.1);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        if (leftDistanceSensorDisconnected()) {
            try {
                throw new Exception("left distance sensor disconnected");
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        if (rightDistanceSensorDisconnected()) {
            try {
                throw new Exception("right distance sensor disconnected");
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        if (colorSensorDisconnected()) {
            try {
                throw new Exception("color sensor disconnected");
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_CLOSED);

        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);

        try {
            sleep(2500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        scanTimer.resetTimer();
    }

    @Override
    public void init_loop() {
        if (scanTimer.getElapsedTime() > 750) {
            navigation = teamPropPipeline.getNavigation();
            telemetry.addData("Navigation:", navigation);
            telemetry.update();
            scanTimer.resetTimer();
        } else if (scanTimer.getElapsedTime() > 720){
            visionPortal.setProcessorEnabled(teamPropPipeline, true);
        } else {
            visionPortal.setProcessorEnabled(teamPropPipeline, false);
        }
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
        twoPersonDrive.frameTimer.resetTimer();
        setBackdropGoalPose();
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */