package org.firstinspires.ftc.teamcode.util;

public class Timer {
    private long startTime;

    public Timer() {
        startTime = System.currentTimeMillis();
    }

    public void resetTimer() {
        startTime = System.currentTimeMillis();
    }

    public long getElapsedTime() {
        return System.currentTimeMillis() - startTime;
    }

    public long getElapsedTimeSeconds() {
        return (long)(getElapsedTime() / 1000.0);
    }
}
