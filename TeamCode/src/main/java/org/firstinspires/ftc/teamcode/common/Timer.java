package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.EventListener;
import java.util.concurrent.TimeUnit;

public class Timer {
    public ElapsedTime elapsedTime;
    private double lastTime;
    private double deltaTime = 0;
    private double currentTime = lastTime;

    public Timer() {
        elapsedTime = new ElapsedTime();
        lastTime = elapsedTime.startTime();

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
        return elapsedTime.startTime() / 10000d;
    }

    public double resetTime() {
        currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) * 1000000.0d;
        elapsedTime.reset();
        return currentTime;
    }
}
