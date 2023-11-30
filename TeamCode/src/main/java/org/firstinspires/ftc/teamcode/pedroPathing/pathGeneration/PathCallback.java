package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import org.firstinspires.ftc.teamcode.util.SingleRunAction;

public class PathCallback extends SingleRunAction {

    private double startCondition;

    private int type, index;

    public static final int TIME = 0, PARAMETRIC = 1;

    public PathCallback(double startCondition, Runnable runnable, int type, int index) {
        super(runnable);
        this.startCondition = startCondition;
        this.type = type;
        if (this.type != TIME || this.type != PARAMETRIC) {
            this.type = PARAMETRIC;
        }
        if (this.type == TIME && this.startCondition < 0) {
            this.startCondition = 0.0;
        }
        if (this.type == PARAMETRIC) {
            this.startCondition = MathFunctions.clamp(this.startCondition, 0, 1);
        }
        this.index = index;
    }

    public int getType() {
        return type;
    }

    public double getStartCondition() {
        return startCondition;
    }

    public int getIndex() {
        return index;
    }
}
