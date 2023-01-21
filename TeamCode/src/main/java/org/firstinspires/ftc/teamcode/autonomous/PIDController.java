package org.firstinspires.ftc.teamcode.autonomous;

public class PIDController {
    private double kp, ki, kd;
    private double errorSum = 0, lastError = 0;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double calculate(double target, double current, double dt) {
        double error = target - current;
        errorSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;
        return kp * error + ki * errorSum + kd * derivative;
    }
}
