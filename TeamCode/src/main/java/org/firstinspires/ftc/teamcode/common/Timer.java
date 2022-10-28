package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Timer {
    public ElapsedTime elapsedTime;
    private double lastTime = elapsedTime.startTime();
    private double deltaTime = 0;
    private double currentTime = lastTime;

    public Timer() {
        elapsedTime = new ElapsedTime();
    }

    public double getTime() {
        currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) * 1000000.0d;
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        return currentTime;
    }

    public double getDeltaTime() {
        getTime();
        return deltaTime;
    }

    public double getStartTime() {
        return elapsedTime.startTime();
    }

    public double resetTime() {
        currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) * 1000000.0d;
        elapsedTime.reset();
        return currentTime;
    }
}
