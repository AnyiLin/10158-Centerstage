package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {
    private DcMotorEx motor;
    private double previousPosition;
    private double currentPosition;
    private double multiplier;

    public final static double FORWARD = 1, REVERSE = -1;

    public Encoder(DcMotorEx setMotor) {
        motor = setMotor;
        multiplier = FORWARD;
        reset();
    }

    public void setDirection(double setMultiplier) {
        multiplier = setMultiplier;
    }

    public void reset() {
        previousPosition = motor.getCurrentPosition();
        currentPosition = motor.getCurrentPosition();
    }

    public void update() {
        previousPosition = currentPosition;
        currentPosition = motor.getCurrentPosition();
    }

    public double getDeltaPosition() {
        return multiplier * (currentPosition - previousPosition);
    }
}
