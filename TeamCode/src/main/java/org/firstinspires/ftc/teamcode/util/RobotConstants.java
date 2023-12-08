package org.firstinspires.ftc.teamcode.util;

public class RobotConstants {
    public static FeedForwardConstant
            liftGravityConstant = (double i) -> {return i;};

    public static CustomPIDFCoefficients

    // lift PIDF coefficients
    liftPIDFCoefficients = new CustomPIDFCoefficients(
            0,
            0,
            0,
            liftGravityConstant),

    // extension PIDF coefficients
    extensionPIDFCoefficients = new CustomPIDFCoefficients(
        0,
        0,
        0,
        0);

    public static int

    // max extension of the scoring slides
    LIFT_MAX_POSITION = 2000, // TODO: check this later

    // max extension of the extension slides
    EXTENSION_MAX_POSITION = 0, // TODO: check this later

    // position the extension slides extend to to avoid the outtake coming back in
    EXTENSION_AVOID_POSITION = 100; // TODO: check this later

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
    INTAKE_AVOID = 8;

    /**
     * IMPORTANT: all arm servo positions are from the left side
     */
    public static double

    // how offset the right intake arm is from the left
    RIGHT_INTAKE_ARM_OFFSET = 0,

    // position for the intake arm being in the robot
    INTAKE_ARM_IN_POSITION = 0, // TODO: SET LATER

    // position for the intake arm being out of the robot at ground level
    INTAKE_ARM_OUT_POSITION = 0, // TODO: SET LATER

    // position for the intake arm being at the top of a pixel stack
    INTAKE_ARM_STACK_TOP_POSITION = 0, // TODO: SET LATER

    // position for the intake arm being at the middle of a pixel stack
    INTAKE_ARM_STACK_MIDDLE_POSITION = 0, // TODO: SET LATER


    // how offset the right outtake arm is from the left
    RIGHT_OUTTAKE_ARM_OFFSET = 0,

    // position for the outtake arm being in the robot
    OUTTAKE_ARM_IN_POSITION = 0, // TODO: SET LATER

    // position for the outtake arm being out of the robot
    OUTTAKE_ARM_OUT_POSITION = 0, // TODO: SET LATER

    // position for the outtake arm on the first tap of a preset
    OUTTAKE_ARM_PRESET_HOLD_POSITION = 0, // TODO: SET LATER


    // the position the wrist has to be at to be vertical when the outtake arm is in the robot
    OUTTAKE_WRIST_VERTICAL_OFFSET = 0,


    // conversion factor of servo position units to degrees for the intake arm
    INTAKE_SERVO_TO_DEGREES = 0, // TODO: SET LATER

    // conversion factor of servo position units to degrees for the outtake arm
    OUTTAKE_ARM_SERVO_TO_DEGREES = 0, // TODO: SET LATER

    // conversion factor of servo position units to degrees for the outtake wrist
    OUTTAKE_WRIST_SERVO_TO_DEGREES = 0, // TODO: SET LATER

    // conversion factor of degrees to servo position units for the intake arm
    INTAKE_DEGREES_TO_SERVO = 0, // TODO: SET LATER

    // conversion factor of degrees to servo position units for the outtake arm
    OUTTAKE_ARM_DEGREES_TO_SERVO = 0, // TODO: SET LATER

    // conversion factor of degrees to servo position units for the outtake wrist
    OUTTAKE_WRIST_DEGREES_TO_SERVO = 0, // TODO: SET LATER


    // open position of the outer scoring claw
    OUTER_OUTTAKE_CLAW_OPEN = 0, // TODO: SET LATER

    // closed position of the outer scoring claw
    OUTER_OUTTAKE_CLAW_CLOSED = 0, // TODO: SET LATER

    // open position of the inner scoring claw
    INNER_OUTTAKE_CLAW_OPEN = 0, // TODO: SET LATER

    // closed position of the inner scoring claw
    INNER_OUTTAKE_CLAW_CLOSED = 0, // TODO: SET LATER

    // open position of the intake claw
    INTAKE_CLAW_OPEN = 0, // TODO: SET LATER

    // closed position of the intake claw
    INTAKE_CLAW_CLOSED = 0, // TODO: SET LATER

    // plane launcher launch position
    PLANE_LAUNCHER_LAUNCH = 0,

    // plane launcher hold position
    PLANE_LAUNCHER_HOLD = 0.2;
}