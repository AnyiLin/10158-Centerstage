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
    LIFT_LOW_PRESET_POSITION = 850, // TODO: set this later

    // max extension of the scoring slides
    LIFT_MIDDLE_PRESET_POSITION = 1400, // TODO: set this later

    // max extension of the scoring slides
    LIFT_HIGH_PRESET_POSITION = 2400, // TODO: set this later

    // outtake arm fine adjust speed in degrees per second
    OUTTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND = 120,

    // intake arm fine adjust speed in degrees per second
    INTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND = 40,

    // outtake wrist fine adjust speed in degrees per second
    OUTTAKE_WRIST_FINE_ADJUST_DEGREES_PER_SECOND = 60,

    // intake arm speed for preset movements in degrees per second
    INTAKE_ARM_PRESET_SPEED = 300;


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
    TRANSFER_GRAB = 15;


    /**
     * IMPORTANT: all arm servo positions are from the left side
     */
    public static double

    // how offset the right intake arm is from the left
    RIGHT_INTAKE_ARM_OFFSET = 0,

    // position for the intake arm being in the robot
    INTAKE_ARM_IN_POSITION = 0.7, // TODO: SET LATER

    // position for the intake arm to avoid the outtake arm
    INTAKE_ARM_AVOID_POSITION = 0.6, // todo set this

    // position for the intake arm being out of the robot at ground level
    INTAKE_ARM_OUT_POSITION = 0.185,

    // position for the intake arm being at the top of a pixel stack
    INTAKE_ARM_STACK_TOP_POSITION = 0.237,

    // position for the intake arm being at the middle of a pixel stack
    INTAKE_ARM_STACK_MIDDLE_POSITION = 0.204,

    // upper bound for intake arm fine adjustment
    INTAKE_ARM_FINE_ADJUST_UPPER_BOUND = 0.5,

    // lower bound for intake arm fine adjustment
    INTAKE_ARM_FINE_ADJUST_LOWER_BOUND = INTAKE_ARM_OUT_POSITION,

    // reference for the vertical position of the intake arm
    INTAKE_ARM_VERTICAL_POSITION = 0.55, // TODO: SET LATER


    // how offset the right outtake arm is from the left
    RIGHT_OUTTAKE_ARM_OFFSET = -0.019,

    // position for the outtake arm being in the robot
    OUTTAKE_ARM_IN_POSITION = 0.9,

    // position for the outtake arm being out of the robot
    OUTTAKE_ARM_OUT_POSITION = 0.2835,

    // upper bound for outtake arm fine adjustment
    OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND = 0.5,

    // lower bound for outtake arm fine adjustment
    OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND = 0.2,


    // the position the wrist has to be at to be vertical when the outtake arm is in the robot
    OUTTAKE_WRIST_VERTICAL_OFFSET = 0.47,

    // upper bound for outtake wrist fine adjustment
    OUTTAKE_WRIST_FINE_ADJUST_UPPER_BOUND = 0, // TODO: SET LATER

    // lower bound for outtake wrist fine adjustment
    OUTTAKE_WRIST_FINE_ADJUST_LOWER_BOUND = 120, // TODO: SET LATER


    // conversion factor of servo position units to degrees for the intake arm
    INTAKE_ARM_SERVO_TO_DEGREES = 355.0,

    // conversion factor of servo position units to degrees for the outtake arm
    OUTTAKE_ARM_SERVO_TO_DEGREES = 270.0,

    // conversion factor of servo position units to degrees for the outtake wrist
    OUTTAKE_WRIST_SERVO_TO_DEGREES = 180.0,

    // conversion factor of degrees to servo position units for the intake arm
    INTAKE_ARM_DEGREES_TO_SERVO = 1.0/355,

    // conversion factor of degrees to servo position units for the outtake arm
    OUTTAKE_ARM_DEGREES_TO_SERVO = 1.0/270,

    // conversion factor of degrees to servo position units for the outtake wrist
    OUTTAKE_WRIST_DEGREES_TO_SERVO = 1.0/355,


    // open position of the outer scoring claw
    OUTER_OUTTAKE_CLAW_OPEN = 0.45,

    // closed position of the outer scoring claw
    OUTER_OUTTAKE_CLAW_CLOSED = 0.27,

    // open position of the inner scoring claw
    INNER_OUTTAKE_CLAW_OPEN = 0.41,

    // closed position of the inner scoring claw
    INNER_OUTTAKE_CLAW_CLOSED = 0.57,

    // open position of the intake claw
    INTAKE_CLAW_OPEN = 0.76,

    // open position of the intake claw used for the transfer
    INTAKE_CLAW_TRANSFER_OPEN = 0.62, // todo test later

    // closed position of the intake claw
    INTAKE_CLAW_CLOSED = 0.58,

    // plane launcher launch position
    PLANE_LAUNCHER_LAUNCH = 0.5,

    // up position for the gate
    GATE_UP = 0.5, // todo set this later

    // down position for the gate
    GATE_DOWN = 0.5, // todo set this later

    // plane launcher hold position
    PLANE_LAUNCHER_HOLD = 0.23,

    // the length from the middle of the robot to the back
    ROBOT_BACK_LENGTH = 10.5;

    /**
     * These are all in milliseconds!
     */
    public static long

    // the time it takes for the outtake arm to go in from the out position back into the robot
    // timing should be the same both ways the arm moves though
    OUTTAKE_ARM_OUT_IN_TIME = 1000,

    // roughly the time it takes for our outtake claws to close
    OUTTAKE_CLAW_CLOSE_TIME = 250,

    // roughly the time it takes for our intake claw to close
    INTAKE_CLAW_CLOSE_TIME = 250,

    // the delay between the inner outtake claw closing and the outer outtake claw closing on transfer
    OUTER_OUTTAKE_CLAW_TRANSFER_DELAY = 35,

    // the delay after putting the intake arm in to transfer
    INTAKE_ARM_IN_TRANSFER_WAIT = 250,

    // roughly the time it takes for our outtake claws to drop something
    OUTTAKE_CLAW_DROP_TIME = 200,

    // the time it takes for the pixels to fall through the transfer
    TRANSFER_DROP_TIME = 300; // TODO: find this later

    public static CustomPIDFCoefficients

    // lift PIDF coefficients
    liftPIDFCoefficients = new CustomPIDFCoefficients(
            0.01 ,
            0,
            -0.000001,
            0);
}