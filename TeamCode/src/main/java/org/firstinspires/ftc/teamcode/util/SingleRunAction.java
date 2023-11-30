package org.firstinspires.ftc.teamcode.util;

public class SingleRunAction {
    private boolean hasBeenRun;

    private Runnable runnable;

    public SingleRunAction(Runnable runnable) {
        this.runnable = runnable;
    }

    public boolean hasBeenRun() {
        return hasBeenRun;
    }

    public boolean run() {
        if (!hasBeenRun) {
            hasBeenRun = true;
            runnable.run();
            return true;
        }
        return false;
    }

    public void reset() {
        hasBeenRun = false;
    }
}
