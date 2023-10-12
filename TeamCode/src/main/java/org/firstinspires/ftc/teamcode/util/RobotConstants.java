package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RobotConstants {
    // Put all the constants we need across opmodes here. Don't forget to make them public and static!

    // gobilda 312 rpms apparently have 537.7 ppr at the output shaft and 28 counts on the actual motor shaft like most other motors

    // the slides are on a 4 to 1 gearbox, so their 28 encoder counts at end of motor are actually 28*4
    // absolute max lift extension is 508.
    public static final int
            LIFT_VELOCITY = 1200,
            LIFT_MAX = 520,
            LIFT_TOLERANCE = 4,
            LIFT_VELOCITY_TOLERANCE = 28*4/6,
            LIFT_GRAB_TOLERANCE = 4,
            TOP_LINE_POSITION = 0,
            MIDDLE_LINE_POSITION = 0,
            BOTTOM_LINE_POSITION = 0,
            LIFT_GRAB_POSITION = 43,
            INTAKE_VELOCITY = 2790; // this is 850 rpm for the gobilda 312 rpm motor. 850/60/2.73 * 537.7

    public static final double
            RIGHT_INTAKE_OFFSET = 0.03,
            LEFT_INTAKE_OUT_POSITION = 0.858,
            RIGHT_INTAKE_OUT_POSITION = 1-LEFT_INTAKE_OUT_POSITION+RIGHT_INTAKE_OFFSET,
            LEFT_INTAKE_OUT_PAUSE_POSITION = 0.83,
            RIGHT_INTAKE_OUT_PAUSE_POSITION = 1-LEFT_INTAKE_OUT_PAUSE_POSITION+RIGHT_INTAKE_OFFSET,
            LEFT_INTAKE_IN_POSITION = 0.3,
            RIGHT_INTAKE_IN_POSITION = 1-LEFT_INTAKE_IN_POSITION+RIGHT_INTAKE_OFFSET,
            LEFT_INTAKE_MIDDLE_POSITION = 0.5,
            RIGHT_INTAKE_MIDDLE_POSITION = 1-LEFT_INTAKE_MIDDLE_POSITION+RIGHT_INTAKE_OFFSET,
            LEFT_INTAKE_DROP_POSITION = 0.1,
            RIGHT_INTAKE_DROP_POSITION = 1-LEFT_INTAKE_DROP_POSITION+RIGHT_INTAKE_OFFSET,

            // for the intake, 0.31 servo position change means 90 degrees forward change
            // so, 0.00344444444 servo position change is a 1 degree change
            INTAKE_SERVO_TO_DEGREES = 1/0.003444444444444444444444444444444444444444,
            INTAKE_DEGREES_TO_SERVO = 0.003444444444444444444444444444444444444444,

            RIGHT_OUTTAKE_OFFSET = 0,
            LEFT_OUTTAKE_OUT_POSITION = 0.45,
            RIGHT_OUTTAKE_OUT_POSITION = 1-LEFT_OUTTAKE_OUT_POSITION+RIGHT_OUTTAKE_OFFSET,
            LEFT_OUTTAKE_IN_POSITION= 0.83,
            RIGHT_OUTTAKE_IN_POSITION = 1-LEFT_OUTTAKE_IN_POSITION+RIGHT_OUTTAKE_OFFSET,
            LEFT_OUTTAKE_GRAB_POSITION= 0.97,
            RIGHT_OUTTAKE_GRAB_POSITION = 1-LEFT_OUTTAKE_GRAB_POSITION+RIGHT_OUTTAKE_OFFSET,
            LEFT_OUTTAKE_AVOID_POSITION = 0.6,
            RIGHT_OUTTAKE_AVOID_POSITION = 1-LEFT_OUTTAKE_AVOID_POSITION+RIGHT_OUTTAKE_OFFSET,
            OUTTAKE_PICK_UP_DEGREES_PER_SECOND = 140,


            // the rev smart servo has a 270 degree range of motion and servo positions rang from [0,1]
            // so, 0.003703703704 servo units is equal to one degree
            OUTTAKE_SERVO_TO_DEGREES = 1/0.003703703704,
            OUTTAKE_DEGREES_TO_SERVO = 0.003703703704,


            //Claw Close positions
            INNER_CLAW_CLOSE_POSITION = 0.45,
            OUTER_CLAW_CLOSE_POSITION = 0.85,

            //Claw Open Positions
            INNER_CLAW_OPEN_POSITION = 0.3,
            OUTER_CLAW_OPEN_POSITION = 1;

    public static final long
            INTAKE_OBSTACLE_OUT_WAIT = 200, // this is in milliseconds
            INTAKE_OBSTACLE_OUT_RETRACT_WAIT = 1000,
            INTAKE_FULL_OUT_WAIT = 1000, // this is in milliseconds
            INTAKE_IN_WAIT = 1000,
            INTAKE_OBSTACLE_IN_WAIT = 500,
            CLAW_GRAB_WAIT = 250, //
            CLAW_CLOSE_WAIT = 200,
            CLAW_LIFT_WAIT = 0,
            LIFT_GO_WAIT = 1000,
            PRESET_TIMEOUT = 1000,
            RESET_PIXEL_DROP_WAIT = 300,
            RESET_FOLD_IN_WAIT = 1000,
            OUTTAKE_OBSTACLE_FOLD_IN_WAIT = 500;

    public static final PIDFCoefficients
            LIFT_UP_VELOCITY_PIDF_COEFFICIENTS = new PIDFCoefficients(16.5,3.25,0,6.75),
            LIFT_UP_POSITION_PIDF_COEFFICIENTS = new PIDFCoefficients(20,0,0,0),
            LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS = new PIDFCoefficients(10,3,0,0),
            LIFT_DOWN_POSITION_PIDF_COEFFICIENTS = new PIDFCoefficients(6,0,0,0);
}
