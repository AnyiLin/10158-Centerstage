package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    public static int

    // max extension of the scoring slides
    LIFT_MAX_POSITION = 2700, // TODO: check this later

    // the upper limit of the lift encoder to start transferring
    LIFT_TRANSFER_UPPER_LIMIT = 20, // TODO: set this later

    // max extension of the scoring slides
    LIFT_LOW_PRESET_POSITION = 0, // TODO: set this later

    // max extension of the scoring slides
    LIFT_MIDDLE_PRESET_POSITION = 0, // TODO: set this later

    // max extension of the scoring slides
    LIFT_HIGH_PRESET_POSITION = 1400, // TODO: set this later

    // outtake arm fine adjust speed in degrees per second
    OUTTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND = 90,

    // intake arm fine adjust speed in degrees per second
    INTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND = 40,

    // outtake wrist fine adjust speed in degrees per second
    OUTTAKE_WRIST_FINE_ADJUST_DEGREES_PER_SECOND = 60,

    // intake arm speed for preset movements in degrees per second
    INTAKE_ARM_PRESET_SPEED = 300,

    // outtake arm speed for preset movements in degrees per second
    OUTTAKE_ARM_PRESET_SPEED = 260;


    public static final int
    // states for mechanisms
    INTAKE_IN = 0,
    INTAKE_OUT = 1,
    OUTTAKE_IN = 2,
    OUTTAKE_GOING_OUT = 3,
    OUTTAKE_OUT = 4,
    OUTTAKE_GOING_IN = 5,
    INTAKE_AVOID = 6,
    OUTTAKE_WAIT = 7,
    TRANSFER_IDLE = 8,
    TRANSFER_POSITIONING = 9,
    TRANSFER_DROPPING = 10,
    TRANSFER_PRESET_HOLD = 11,
    TRANSFER_OUT = 12,
    TRANSFER_RESET = 13,
    TRANSFER_RESET_CLAW_DROP = 14,
    TRANSFER_GRAB = 15,
    TRANSFER_INTAKE_AVOID = 16,
    TRANSFER_INTAKE_AVOID_RUN_OUT = 17;


    /**
     * IMPORTANT: all arm servo positions are from the left side
     */
    public static double

    // how offset the right intake arm is from the left
    RIGHT_INTAKE_ARM_OFFSET = 0,

    // position for the intake arm being in the robot
    INTAKE_ARM_IN_POSITION = 0.1925,

    // position for the intake arm to avoid the outtake arm
    INTAKE_ARM_AVOID_POSITION = 0.4825,

    // position for the intake arm to avoid the beacon in auto
    INTAKE_ARM_AUTO_AVOID_POSITION = 0.5025, // todo set this

    // position for the intake arm being out of the robot at ground level
    INTAKE_ARM_OUT_POSITION = 0.842,

    // position for the intake arm being at the top of a pixel stack
    INTAKE_ARM_STACK_TOP_POSITION = 0.767,

    // position for the intake arm being at the middle of a pixel stack
    INTAKE_ARM_STACK_MIDDLE_POSITION = 0.8085,

    // upper bound for intake arm fine adjustment
    INTAKE_ARM_FINE_ADJUST_UPPER_BOUND = 0.4825,

    // lower bound for intake arm fine adjustment
    INTAKE_ARM_FINE_ADJUST_LOWER_BOUND = INTAKE_ARM_OUT_POSITION,

    // reference for the vertical position of the intake arm
    INTAKE_ARM_VERTICAL_POSITION = 0.3655,//0.283, // TODO: SET LATER


    // how offset the right outtake arm is from the left
    RIGHT_OUTTAKE_ARM_OFFSET = -0.01,

    // position for the outtake arm being in the robot
    OUTTAKE_ARM_IN_POSITION = 0.82,

    // position for the outtake arm to drop in the robot
    OUTTAKE_ARM_DROP_POSITION = 0.84,

    // position for the outtake arm being out of the robot
    OUTTAKE_ARM_OUT_POSITION = 0.465,

    // upper bound for outtake arm fine adjustment
    OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND = 0.465,

    // lower bound for outtake arm fine adjustment
    OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND = 0.2,

    // outtake arm yellow pixel scoring position
    OUTTAKE_ARM_YELLOW_SCORE_POSITION = 0.362,

    // outtake arm cycle pixel scoring position for first pixel
    OUTTAKE_ARM_CYCLE_FIRST_SCORE_POSITION = 0.386,

    // outtake arm cycle pixel scoring position for second pixel
    OUTTAKE_ARM_CYCLE_SECOND_SCORE_POSITION = 0.375,


    // the position the wrist has to be at to be vertical when the outtake arm is in the robot
    OUTTAKE_WRIST_VERTICAL_OFFSET = 0.845,

    // upper bound for outtake wrist fine adjustment
    OUTTAKE_WRIST_FINE_ADJUST_UPPER_BOUND = -60,

    // lower bound for outtake wrist fine adjustment
    OUTTAKE_WRIST_FINE_ADJUST_LOWER_BOUND = 180,


    // conversion factor of servo position units to degrees for the intake arm
    INTAKE_ARM_SERVO_TO_DEGREES = 270.0,

    // conversion factor of servo position units to degrees for the outtake arm
    OUTTAKE_ARM_SERVO_TO_DEGREES = 355.0,

    // conversion factor of servo position units to degrees for the outtake wrist
    OUTTAKE_WRIST_SERVO_TO_DEGREES = 355.0,

    // conversion factor of degrees to servo position units for the intake arm
    INTAKE_ARM_DEGREES_TO_SERVO = 1.0/270,

    // conversion factor of degrees to servo position units for the outtake arm
    OUTTAKE_ARM_DEGREES_TO_SERVO = 1.0/355,

    // conversion factor of degrees to servo position units for the outtake wrist
    OUTTAKE_WRIST_DEGREES_TO_SERVO = 1.0/355,


    // open position of the outer scoring claw
    OUTER_OUTTAKE_CLAW_OPEN = 0.45,

    // closed position of the outer scoring claw
    OUTER_OUTTAKE_CLAW_CLOSED = 0.27,

    // open position of the inner scoring claw
    INNER_OUTTAKE_CLAW_OPEN = 0.67,

    // closed position of the inner scoring claw
    INNER_OUTTAKE_CLAW_CLOSED = 0.88,

    // open position of the intake claw
    INTAKE_CLAW_OPEN = 0.76,

    // closed position of the intake claw
    INTAKE_CLAW_CLOSED = 0.58,

    // plane launcher launch position
    PLANE_LAUNCHER_LAUNCH = 0.3,

    // plane launcher hold position
    PLANE_LAUNCHER_HOLD = 0.07,

    // the length from the middle of the robot to the back
    ROBOT_BACK_LENGTH = 11.5,

    // the length from the middle of the robot to an extended intake arm
    ROBOT_FRONT_LENGTH = 16.25;

    /**
     * These are all in milliseconds!
     */
    public static long

    // the time it takes for the outtake arm to go in from the out position back into the robot
    // timing should be the same both ways the arm moves though
    MIN_OUTTAKE_ARM_OUT_IN_TIME = 1000,
    MAX_OUTTAKE_ARM_OUT_IN_TIME = 1000,

    // roughly the time it takes for our outtake claws to close
    OUTTAKE_CLAW_CLOSE_TIME = 250,

    // roughly the time it takes for our intake claw to close
    INTAKE_CLAW_CLOSE_TIME = 250,

    // the delay between the inner outtake claw closing and the outer outtake claw closing on transfer
    TRANSFER_CLAW_DELAY = 50,

    // the delay for the outtake arm to drop during the transfer
    OUTTAKE_ARM_DROP_TIME = 200,

    // the delay after putting the intake arm in to transfer
    INTAKE_ARM_IN_TRANSFER_WAIT = 250,

    // delay for instant out transfer
    TRANSFER_INSTANT_OUT_DELAY = 250,

    // roughly the time it takes for our outtake claws to drop something
    OUTTAKE_CLAW_DROP_TIME = 150,

    // the time it takes for the pixels to fall through the transfer
    TRANSFER_DROP_TIME = 200; // TODO: find this later

    public static CustomPIDFCoefficients

    // lift PIDF coefficients
    liftPIDFCoefficients = new CustomPIDFCoefficients(
            0.006,
            0,
            0.00027,
            0);
}