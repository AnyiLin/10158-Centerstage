package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    public static int

    // max extension of the scoring slides
    LIFT_MAX_POSITION = 2700, // TODO: check this later

    // the upper limit of the lift encoder to start transferring
    LIFT_TRANSFER_UPPER_LIMIT = 20, // TODO: set this later

    // the upper limit of the lift encoder to start transferring
    EXTENSION_TRANSFER_UPPER_LIMIT = 10, // TODO: set this later

    // max extension of the scoring slides
    LIFT_LOW_PRESET_POSITION = 850, // TODO: set this later

    // max extension of the scoring slides
    LIFT_MIDDLE_PRESET_POSITION = 1400, // TODO: set this later

    // max extension of the scoring slides
    LIFT_HIGH_PRESET_POSITION = 2400, // TODO: set this later

    // max extension of the extension slides
    EXTENSION_MAX_POSITION = 1600, // TODO: check this later

    // position the extension slides extend to to avoid the outtake coming back in
    EXTENSION_AVOID_POSITION = 130, // TODO: set this later

    // a small buffer on the extension avoid position to make it work better
    EXTENSION_AVOID_POSITION_BUFFER = 10,

    // a the maximum activation range of the extension reset
    EXTENSION_ZERO_RESET_LIMIT = 20,

    // the speed at which the extension target position is manually adjusted in ticks per second
    EXTENSION_MANUAL_ADJUST_SPEED = 200,

    // outtake arm fine adjust speed in degrees per second
    OUTTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND = 120,

    // intake arm fine adjust speed in degrees per second
    INTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND = 40,

    // outtake wrist fine adjust speed in degrees per second
    OUTTAKE_WRIST_FINE_ADJUST_DEGREES_PER_SECOND = 60,

    // intake arm speed for preset movements in degrees per second
    INTAKE_ARM_PRESET_SPEED = 180;


    public static final int
    // states for mechanisms
    INTAKE_IN = 0,
    INTAKE_GOING_OUT = 1,
    INTAKE_OUT = 2,
    INTAKE_GOING_IN = 3,
    OUTTAKE_IN = 4,
    OUTTAKE_GOING_OUT = 5,
    OUTTAKE_OUT = 6,
    OUTTAKE_GOING_IN = 7,
    INTAKE_AVOID = 8,
    //OUTTAKE_PRESET = 9,
    OUTTAKE_WAIT = 10,
    EXTENSION_NOMINAL = 11,
    EXTENSION_AVOID = 12,
    EXTENSION_AVOID_RESET = 13,
    //OUTTAKE_MOVING_OUTSIDE = 14,
    INTAKE_IDLE = 15,
    OUTTAKE_IDLE = 16,
    TRANSFER_IDLE = 17,
    TRANSFER_POSITIONING = 18,
    TRANSFER_TRANSFERRING = 19,
    TRANSFER_PRESET_HOLD = 20,
    TRANSFER_OUT = 21,
    TRANSFER_RESET = 22,
    TRANSFER_RESET_CLAW_DROP = 23,
    TRANSFER_GRAB = 24,
    EXTENSION_ZERO_RESET = 25,
    EXTENSION_ZERO_RESET_READY = 26;


    /**
     * IMPORTANT: all arm servo positions are from the left side
     */
    public static double

    // how offset the right intake arm is from the left
    RIGHT_INTAKE_ARM_OFFSET = 0,

    // position for the intake arm being in the robot
    INTAKE_ARM_IN_POSITION = 0.794, // TODO: SET LATER

    // position for the intake arm being out of the robot at ground level
    INTAKE_ARM_OUT_POSITION = 0.165,

    // position for the intake arm being at the top of a pixel stack
    INTAKE_ARM_STACK_TOP_POSITION = 0.191,

    // position for the intake arm being at the middle of a pixel stack
    INTAKE_ARM_STACK_MIDDLE_POSITION = 0.173,

    // upper bound for intake arm fine adjustment
    INTAKE_ARM_FINE_ADJUST_UPPER_BOUND = 0.5, // TODO: SET LATER

    // lower bound for intake arm fine adjustment
    INTAKE_ARM_FINE_ADJUST_LOWER_BOUND = 0.165, // TODO: SET LATER


    // how offset the right outtake arm is from the left
    RIGHT_OUTTAKE_ARM_OFFSET = -0.025,

    // position for the outtake arm being in the robot
    OUTTAKE_ARM_IN_POSITION = 0.9,

    // position for the outtake arm being out of the robot
    OUTTAKE_ARM_OUT_POSITION = 0.38,

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
    OUTTAKE_WRIST_DEGREES_TO_SERVO = 1.0/180,


    // open position of the outer scoring claw
    OUTER_OUTTAKE_CLAW_OPEN = 0.45,

    // closed position of the outer scoring claw
    OUTER_OUTTAKE_CLAW_CLOSED = 0.27,

    // open position of the inner scoring claw
    INNER_OUTTAKE_CLAW_OPEN = 0.4,

    // closed position of the inner scoring claw
    INNER_OUTTAKE_CLAW_CLOSED = 0.57,

    // open position of the intake claw
    INTAKE_CLAW_OPEN = 0.76,

    // closed position of the intake claw
    INTAKE_CLAW_CLOSED = 0.5,

    // plane launcher launch position
    PLANE_LAUNCHER_LAUNCH = 0,

    // plane launcher hold position
    PLANE_LAUNCHER_HOLD = 0.2,

    // the length from the middle of the robot to the back
    ROBOT_BACK_LENGTH = 15;

    /**
     * These are all in milliseconds!
     */
    public static long

    // the time it takes for the outtake arm to go in from the out position back into the robot
    // timing should be the same both ways the arm moves though
    OUTTAKE_ARM_OUT_IN_TIME = 1000,

    // roughly the time it takes for our outtake claws to close
    OUTTAKE_CLAW_CLOSE_TIME = 250,

    // the delay between the inner outtake claw closing and the outer outtake claw closing on transfer
    OUTER_OUTTAKE_CLAW_TRANSFER_DELAY = 50,

    // roughly the time it takes for our outtake claws to drop something
    OUTTAKE_CLAW_DROP_TIME = 200,

    // the time it takes for the pixels to fall through the transfer
    TRANSFER_DROP_TIME = 300, // TODO: find this later

    // the time allowed for the backwards running of the motors during extension zero reset
    EXTENSION_ZERO_RESET_TIME = 500;

    public static CustomPIDFCoefficients

    // lift PIDF coefficients
    liftPIDFCoefficients = new CustomPIDFCoefficients(
            0.011,
            0,
            0.0001,
            0),

    // extension PIDF coefficients
    extensionPIDFCoefficients = new CustomPIDFCoefficients(
            0.01,
            0.01,
            0.001,
            0);


}