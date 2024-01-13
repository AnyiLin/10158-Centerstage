package org.firstinspires.ftc.teamcode.competition.autonomous;


import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_STACK_MIDDLE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_STACK_TOP_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_MIDDLE_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CLAW_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_POSITIONING;
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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.util.SingleRunAction;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.VisionPortalTeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Autonomous(name = "Red Right Inner Auto", group = "Autonomous")
public class RedRightInnerAuto extends OpMode {

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

    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose, thirdCycleStackPose, thirdCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(63+72, 12+72, Math.PI);

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop, firstCycleToStack, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleScoreOnBackdrop, thirdCycleToStack, thirdCycleScoreOnBackdrop;

    private double scoreSpikeMarkAngle;

    private int pathState, EXTENSION_SPIKE_MARK_POSITION;

    public void setBackdropGoalPose() {
        switch (navigation) {
            case "left":
                spikeMarkGoalPose = new Pose2d(redRightSideLeftSpikeMark.getX() - 8, redRightSideLeftSpikeMark.getY(), Math.PI/2);
                EXTENSION_SPIKE_MARK_POSITION = 430;
                initialBackdropGoalPose = new Pose2d(redLeftBackdrop.getX() + 3, +redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                firstCycleStackPose = new Pose2d(redInnerStack.getX(), 84);
                firstCycleBackdropGoalPose = new Pose2d(redRightBackdrop.getX() - 2, redRightBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                secondCycleStackPose = new Pose2d(redInnerStack.getX(), 84);;
                secondCycleBackdropGoalPose = new Pose2d(redRightBackdrop.getX() - 2, redRightBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                thirdCycleStackPose = new Pose2d(redOuterStack.getX(), 84);;
                thirdCycleBackdropGoalPose = new Pose2d(redRightBackdrop.getX() - 2, redRightBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(redRightSideMiddleSpikeMark.getX(), redRightSideMiddleSpikeMark.getY(), Math.PI/2);
                EXTENSION_SPIKE_MARK_POSITION = 305;
                initialBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX() + 2.5, redMiddleBackdrop.getY()-ROBOT_BACK_LENGTH,Math.PI * 1.5);
                firstCycleStackPose = new Pose2d(redInnerStack.getX(), 84);
                firstCycleBackdropGoalPose = new Pose2d(redLeftBackdrop.getX() + 2, redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                secondCycleStackPose = new Pose2d(redInnerStack.getX(), 84);
                secondCycleBackdropGoalPose = new Pose2d(redLeftBackdrop.getX() + 2, redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                thirdCycleStackPose = new Pose2d(redOuterStack.getX(), 84);
                thirdCycleBackdropGoalPose = new Pose2d(redLeftBackdrop.getX() + 2, redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(redRightSideRightSpikeMark.getX() - 10, redRightSideRightSpikeMark.getY(), Math.PI/2);
                EXTENSION_SPIKE_MARK_POSITION = 180;
                initialBackdropGoalPose = new Pose2d(redRightBackdrop.getX() - 3, redRightBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                firstCycleStackPose = new Pose2d(redInnerStack.getX(), 84);
                firstCycleBackdropGoalPose = new Pose2d(redLeftBackdrop.getX() + 2, redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                secondCycleStackPose = new Pose2d(redInnerStack.getX(), 84);
                secondCycleBackdropGoalPose = new Pose2d(redLeftBackdrop.getX() + 2, redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                thirdCycleStackPose = new Pose2d(redOuterStack.getX(), 84);
                thirdCycleBackdropGoalPose = new Pose2d(redLeftBackdrop.getX() + 2, redLeftBackdrop.getY()-ROBOT_BACK_LENGTH, Math.PI * 1.5);
                break;
        }
    }

    public void buildPaths() {
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), new Point(123, 88, Point.CARTESIAN), new Point(121, 101, Point.CARTESIAN)));
        Vector scoreSpikeMarkVector = new Vector();
        scoreSpikeMarkVector.setOrthogonalComponents(spikeMarkGoalPose.getX() - scoreSpikeMark.getLastControlPoint().getX(), spikeMarkGoalPose.getY() - scoreSpikeMark.getLastControlPoint().getY());
        scoreSpikeMarkAngle = scoreSpikeMarkVector.getTheta();
        //scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading(), scoreSpikeMarkVector.getTheta());
        scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());

        initialScoreOnBackdrop = new Path(new BezierCurve(new Point(scoreSpikeMark.getLastControlPoint().getX(), scoreSpikeMark.getLastControlPoint().getY(), Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 111, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
        //initialScoreOnBackdrop.setLinearHeadingInterpolation(scoreSpikeMarkVector.getTheta(), Math.PI * 1.5);
        initialScoreOnBackdrop.setConstantHeadingInterpolation(Math.PI * 1.5);
        initialScoreOnBackdrop.setPathEndTimeout(2.5);


        firstCycleToStack = new Path(new BezierCurve(new Point(initialBackdropGoalPose), new Point(initialBackdropGoalPose.getX(), 100, Point.CARTESIAN), new Point(72, 125, Point.CARTESIAN), new Point(firstCycleStackPose)));
        firstCycleToStack.setConstantHeadingInterpolation(Math.PI * 1.5);

        firstCycleScoreOnBackdrop = new Path(new BezierCurve(new Point(firstCycleStackPose), new Point(72, 135, Point.CARTESIAN), new Point(firstCycleBackdropGoalPose.getX(), 100, Point.CARTESIAN), new Point(firstCycleBackdropGoalPose)));
        firstCycleScoreOnBackdrop.setConstantHeadingInterpolation(Math.PI * 1.5);
        firstCycleScoreOnBackdrop.setPathEndTimeout(2.5);


        secondCycleToStack = new Path(new BezierCurve(new Point(firstCycleBackdropGoalPose), new Point(firstCycleBackdropGoalPose.getX(), 100, Point.CARTESIAN), new Point(72, 125, Point.CARTESIAN), new Point(secondCycleStackPose)));
        secondCycleToStack.setConstantHeadingInterpolation(Math.PI * 1.5);

        secondCycleScoreOnBackdrop = new Path(new BezierCurve(new Point(secondCycleStackPose), new Point(72, 135, Point.CARTESIAN), new Point(secondCycleBackdropGoalPose.getX(), 100, Point.CARTESIAN), new Point(secondCycleBackdropGoalPose)));
        secondCycleScoreOnBackdrop.setConstantHeadingInterpolation(Math.PI * 1.5);
        secondCycleScoreOnBackdrop.setPathEndTimeout(2.5);


        thirdCycleToStack = new Path(new BezierCurve(new Point(secondCycleBackdropGoalPose), new Point(secondCycleBackdropGoalPose.getX(), 112, Point.CARTESIAN), new Point(secondCycleStackPose.getX(), 121, Point.CARTESIAN), new Point(secondCycleStackPose)));
        thirdCycleToStack.setConstantHeadingInterpolation(Math.PI * 1.5);

        thirdCycleScoreOnBackdrop = new Path(new BezierCurve(new Point(thirdCycleStackPose), new Point(thirdCycleStackPose.getX(), 121, Point.CARTESIAN), new Point(thirdCycleBackdropGoalPose.getX(), 112, Point.CARTESIAN), new Point(thirdCycleBackdropGoalPose)));
        thirdCycleScoreOnBackdrop.setConstantHeadingInterpolation(Math.PI * 1.5);
        thirdCycleScoreOnBackdrop.setPathEndTimeout(2.5);
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
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_OPEN);
                    setPathState(2);
                }
                break;
            case 2: // moves mechanisms into position to score and pick up from stack as well as starts moving to score
                if (pathTimer.getElapsedTime() > 100) {
                    //twoPersonDrive.setExtensionTargetPosition(0);
                    twoPersonDrive.moveOuttake(OUTTAKE_OUT);
                    twoPersonDrive.setLiftTargetPosition(LIFT_MIDDLE_PRESET_POSITION);
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(3);
                }
                break;
            case 3: // detects for end of the path and outtake out and drops pixel
                if (pathTimer.getElapsedTime() > 400) {
                    moveIntakeToAvoidBeaconPosition.run();
                }
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    follower.holdPoint(new BezierPoint(new Point(initialScoreOnBackdrop.getLastControlPoint().getX(), initialScoreOnBackdrop.getLastControlPoint().getY() + 3.25, Point.CARTESIAN)), Math.PI * 1.5);
                    moveIntakeToAvoidBeaconPosition.reset();
                    setPathState(-2);
                }
                break;
            case -2: // detects for end of the path and outtake out and drops pixel
                if (pathTimer.getElapsedTime() > 2000 && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(25);
                    /*
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_OUT_AUTO_AVOID_POSITION);
                    setPathState(4);
                     */
                }
                break;


            case 4: // starts the robot off on to the first stack once the pixels have been dropped
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    follower.followPath(firstCycleToStack);
                    setPathState(5);
                }
                break;
            case 5: // once the robot is in position, then run extension out
                if (!follower.isBusy()) {
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_TOP_POSITION);
                    follower.holdPoint(new BezierPoint(firstCycleToStack.getLastControlPoint()), Math.PI * 1.5);
                    //twoPersonDrive.setExtensionTargetPosition(EXTENSION_MAX_POSITION - 50);
                    setPathState(6);
                }
                break;
            case 6: // once the extension is within like 20 or so or if the velocity is less than some value, then grab
                if (true/*(twoPersonDrive.extensionEncoder.getCurrentPosition() >= EXTENSION_MAX_POSITION - 20) || (twoPersonDrive.extensionEncoder.getVelocity() < 4 && twoPersonDrive.extensionEncoder.getCurrentPosition() > EXTENSION_MAX_POSITION/2)*/) {
                    //twoPersonDrive.setExtensionTargetPosition(twoPersonDrive.extensionEncoder.getCurrentPosition());
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
                    setPathState(7);
                }
                break;
            case 7: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
                if (pathTimer.getElapsedTime() > 200) {
                    follower.breakFollowing();
                    twoPersonDrive.moveIntake(INTAKE_IN);
                    //twoPersonDrive.setExtensionTargetPosition(EXTENSION_MAX_POSITION - 300); // todo adjust this
                    setPathState(8);
                }
                break;
            case 8: // once the intake is in, then pull everything back in and start following the scoring path
                if (twoPersonDrive.intakeState == INTAKE_IN) {
                    //twoPersonDrive.setExtensionTargetPosition(0);
                    twoPersonDrive.liftPresetTargetPosition = LIFT_MIDDLE_PRESET_POSITION;
                    follower.followPath(firstCycleScoreOnBackdrop);
                    setPathState(9);
                }
                break;
            case 9: // detects for end of path and outtake out, and then drops outer pixel as well as putting the intake out
                if (follower.atParametricEnd()) {
                    twoPersonDrive.setTransferState(TRANSFER_POSITIONING);
                    follower.holdPoint(new BezierPoint(new Point(firstCycleScoreOnBackdrop.getLastControlPoint().getX(), firstCycleScoreOnBackdrop.getLastControlPoint().getY() + 2.5, Point.CARTESIAN)), Math.PI * 1.5);
                    setPathState(-3);
                }
                break;
            case -3: // detects for end of the path and outtake out and drops pixel
                if (MathFunctions.roughlyEquals(twoPersonDrive.outerOuttakeClaw.getPosition(), OUTER_OUTTAKE_CLAW_CLOSED) && MathFunctions.roughlyEquals(twoPersonDrive.innerOuttakeClaw.getPosition(), INNER_OUTTAKE_CLAW_CLOSED)) {
                    moveIntakeToAvoidBeaconPosition.run();
                }
                if (twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    moveIntakeToAvoidBeaconPosition.reset();
                    //twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_OUT_AUTO_AVOID_POSITION);
                    setPathState(10);
                }
                break;
            case 10: // once the outer pixel has dropped, drop the inner one and fold up
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(25);
                    break;
                    // todo remove for later
                    //setPathState(11);
                }
                break;


            case 11: // once the inner pixel has dropped, start the robot off to the second pass on the first stack
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME/* && twoPersonDrive.extensionEncoder.getCurrentPosition() < EXTENSION_TRANSFER_UPPER_LIMIT*/) {
                    follower.followPath(secondCycleToStack);
                    setPathState(12);
                }
                break;
            case 12: // once the robot is in position, then run extension out
                if (!follower.isBusy()) {
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_MIDDLE_POSITION);
                    follower.holdPoint(new BezierPoint(secondCycleToStack.getLastControlPoint()), Math.PI * 1.5);
                    //twoPersonDrive.setExtensionTargetPosition(EXTENSION_MAX_POSITION - 50);
                    setPathState(13);
                }
                break;
            case 13: // once the extension is within like 20 or so or if the velocity is less than some value, then grab
                /*
                if ((twoPersonDrive.extensionEncoder.getCurrentPosition() >= EXTENSION_MAX_POSITION - 20) || (twoPersonDrive.extensionEncoder.getVelocity() < 4 && twoPersonDrive.extensionEncoder.getCurrentPosition() > EXTENSION_MAX_POSITION/2)) {
                    twoPersonDrive.setExtensionTargetPosition(twoPersonDrive.extensionEncoder.getCurrentPosition());
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
                    setPathState(14);
                }

                 */
                break;
            case 14: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
                if (pathTimer.getElapsedTime() > 200) {
                    follower.breakFollowing();
                    twoPersonDrive.moveIntake(INTAKE_IN);
                    //twoPersonDrive.setExtensionTargetPosition(EXTENSION_MAX_POSITION - 300); // todo adjust this
                    setPathState(15);
                }
                break;
            case 15: // once the intake is in, then pull everything back in and start following the scoring path
                if (twoPersonDrive.intakeState == INTAKE_IN) {
                    //twoPersonDrive.setExtensionTargetPosition(0);
                    twoPersonDrive.liftPresetTargetPosition = LIFT_MIDDLE_PRESET_POSITION;
                    follower.followPath(secondCycleScoreOnBackdrop);
                    setPathState(16);
                }
                break;
            case 16: // detects for end of path and outtake out, and then drops outer pixel as well as putting the intake out
                if (follower.atParametricEnd()) {
                    twoPersonDrive.setTransferState(TRANSFER_POSITIONING);
                    follower.holdPoint(new BezierPoint(new Point(secondCycleScoreOnBackdrop.getLastControlPoint().getX(), secondCycleScoreOnBackdrop.getLastControlPoint().getY() + 2.5, Point.CARTESIAN)), Math.PI * 1.5);
                    setPathState(-4);
                }
                break;
            case -4: // detects for end of the path and outtake out and drops pixel
                if (MathFunctions.roughlyEquals(twoPersonDrive.outerOuttakeClaw.getPosition(), OUTER_OUTTAKE_CLAW_CLOSED) && MathFunctions.roughlyEquals(twoPersonDrive.innerOuttakeClaw.getPosition(), INNER_OUTTAKE_CLAW_CLOSED)) {
                    moveIntakeToAvoidBeaconPosition.run();
                }
                if (twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    moveIntakeToAvoidBeaconPosition.reset();
                    //twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_OUT_AUTO_AVOID_POSITION);
                    setPathState(17);
                }
                break;
            case 17: // once the outer pixel has dropped, drop the inner one and fold up
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(18);
                }
                break;


            case 18: // once the inner pixel has dropped, start the robot off to the second pass on the first stack
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME /*&& twoPersonDrive.extensionEncoder.getCurrentPosition() < EXTENSION_TRANSFER_UPPER_LIMIT*/) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 22) {
                        setPathState(25);
                        break;
                    }
                    follower.followPath(thirdCycleToStack);
                    setPathState(19);
                }
                break;
            case 19: // once the robot is in position, then run extension out
                if (!follower.isBusy()) {
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_TOP_POSITION);
                    follower.holdPoint(new BezierPoint(thirdCycleToStack.getLastControlPoint()), Math.PI * 1.5);
                    //twoPersonDrive.setExtensionTargetPosition(EXTENSION_MAX_POSITION - 50);
                    setPathState(20);
                }
                break;
            case 20: // once the extension is within like 20 or so or if the velocity is less than some value, then grab
                /*
                if ((twoPersonDrive.extensionEncoder.getCurrentPosition() >= EXTENSION_MAX_POSITION - 20) || (twoPersonDrive.extensionEncoder.getVelocity() < 4 && twoPersonDrive.extensionEncoder.getCurrentPosition() > EXTENSION_MAX_POSITION/2)) {
                    twoPersonDrive.setExtensionTargetPosition(twoPersonDrive.extensionEncoder.getCurrentPosition());
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
                    setPathState(21);
                }

                 */
                break;
            case 21: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
                if (pathTimer.getElapsedTime() > 200) {
                    follower.breakFollowing();
                    twoPersonDrive.moveIntake(INTAKE_IN);
                    //twoPersonDrive.setExtensionTargetPosition(EXTENSION_MAX_POSITION - 300); // todo adjust this
                    setPathState(22);
                }
                break;
            case 22: // once the intake is in, then pull everything back in and start following the scoring path
                if (twoPersonDrive.intakeState == INTAKE_IN) {
                    //twoPersonDrive.setExtensionTargetPosition(0);
                    twoPersonDrive.liftPresetTargetPosition = LIFT_MIDDLE_PRESET_POSITION;
                    follower.followPath(thirdCycleScoreOnBackdrop);
                    setPathState(23);
                }
                break;
            case 23: // detects for end of path and outtake out, and then drops outer pixel as well as putting the intake out
                if (follower.atParametricEnd()) {
                    follower.holdPoint(new BezierPoint(new Point(thirdCycleScoreOnBackdrop.getLastControlPoint().getX(), thirdCycleScoreOnBackdrop.getLastControlPoint().getY() + 3, Point.CARTESIAN)), Math.PI * 1.5);
                    twoPersonDrive.setTransferState(TRANSFER_POSITIONING);
                    setPathState(-5);
                }
                break;
            case -5:
                if (twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    setPathState(24);
                }
            break;
            case 24: // once the outer pixel has dropped, drop the inner one and fold up
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(25);
                }
                break;


            case 25: // move the intake in
                twoPersonDrive.moveIntake(INTAKE_IN);
                setPathState(26);
                break;
            case 26: // once the robot is nice and folded up, request stop
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

        teamPropPipeline = new VisionPortalTeamPropPipeline(0);

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