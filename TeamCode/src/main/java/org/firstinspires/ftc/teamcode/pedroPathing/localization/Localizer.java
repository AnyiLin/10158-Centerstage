package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

public abstract class Localizer {

    public abstract Pose getPose();

    public abstract Pose getVelocity();

    public abstract Vector getVelocityVector();

    public abstract void setStartPose(Pose setStart);

    public abstract void setPose(Pose setPose);

    public abstract void update();
}
