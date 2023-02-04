package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
    private double kp, ki, kd;
    private double kiCap;
    private double errorSum = 0, lastError = 0;


    public PIDController(double kp, double ki, double kd, double kiCap) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kiCap = kiCap;
    }

    public PIDController(PIDCoefficients coefficients) {
        this.kp = coefficients.kp;
        this.ki = coefficients.ki;
        this.kd = coefficients.kd;

        this.kiCap = coefficients.kiCap;
    }

    public double calculate(double target, double current, double deltaTime, Telemetry telemetry) {
        double error = target - current;
        errorSum += error * deltaTime;

        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        double integral = Math.min(ki * errorSum, kiCap);

        telemetry.addLine(String.format("\nProportional component: %4.2f\nDerivative component: %4.2f\nIntegral component: %4.2f\n", error * kp, kd * derivative, integral));

        return kp * error + integral + kd * derivative;
    }

    public void resetError() {
        lastError = 0;
        errorSum = 0;
    }


}