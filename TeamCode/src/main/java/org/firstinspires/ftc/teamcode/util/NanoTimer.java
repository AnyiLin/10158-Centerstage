package org.firstinspires.ftc.teamcode.util;

public class NanoTimer {
    private long startTime;

    public NanoTimer() {
        startTime = System.nanoTime();
    }

    public void resetTimer() {
        startTime = System.nanoTime();
    }

    public long getElapsedTime() {
        return System.nanoTime() - startTime;
    }

    public double getElapsedTimeSeconds() {
        return (getElapsedTime() / Math.pow(10.0,9));
    }
}
