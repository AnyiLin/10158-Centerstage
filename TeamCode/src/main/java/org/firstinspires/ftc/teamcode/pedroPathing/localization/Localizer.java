package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

/**
 * This is the Localizer class.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
public abstract class Localizer {

    public abstract Pose getPose();

    public abstract Pose getVelocity();

    public abstract Vector getVelocityVector();

    public abstract void setStartPose(Pose setStart);

    public abstract void setPose(Pose setPose);

    public abstract void update();

    public abstract double getTotalHeading();
}
