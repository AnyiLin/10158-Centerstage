package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import java.util.ArrayList;

public class PathBuilder {
    private ArrayList<Path> paths = new ArrayList<>();

    private ArrayList<PathCallback> callbacks = new ArrayList<>();

    public PathBuilder() {
    }

    public PathBuilder addPath(Path path) {
        this.paths.add(path);
        return this;
    }

    public PathBuilder addPath(BezierCurve curve) {
        this.paths.add(new Path(curve));
        return this;
    }

    public PathBuilder setLinearHeadingInterpolation(double startHeading, double endHeading) {
        this.paths.get(paths.size()-1).setLinearHeadingInterpolation(startHeading, endHeading);
        return this;
    }

    public PathBuilder setConstantHeadingInterpolation(double setHeading) {
        this.paths.get(paths.size()-1).setConstantHeadingInterpolation(setHeading);
        return this;
    }

    public PathBuilder setReversed(boolean set) {
        this.paths.get(paths.size()-1).setReversed(set);
        return this;
    }

    public PathBuilder setPathEndVelocity(double set) {
        this.paths.get(paths.size()-1).setPathEndVelocity(set);
        return this;
    }

    public PathBuilder setPathEndTranslational(double set) {
        this.paths.get(paths.size()-1).setPathEndTranslational(set);
        return this;
    }

    public PathBuilder setPathEndHeading(double set) {
        this.paths.get(paths.size()-1).setPathEndHeading(set);
        return this;
    }

    public PathBuilder setPathEndTValue(double set) {
        this.paths.get(paths.size()-1).setPathEndTValue(set);
        return this;
    }

    public PathBuilder setPathEndTimeout(double set) {
        this.paths.get(paths.size()-1).setPathEndTimeout(set);
        return this;
    }

    public PathBuilder addTemporalCallback(double time, Runnable runnable) {
        this.callbacks.add(new PathCallback(time, runnable, PathCallback.TIME, paths.size()-1));
        return this;
    }

    public PathBuilder addParametricCallback(double t, Runnable runnable) {
        this.callbacks.add(new PathCallback(t, runnable, PathCallback.PARAMETRIC, paths.size()-1));
        return this;
    }

    public PathChain build() {
        PathChain returnChain = new PathChain(paths);
        returnChain.setCallbacks(callbacks);
        return returnChain;
    }
}
