package org.firstinspires.ftc.teamcode.autonomous;

public class PIDCoefficients {
    public double kp;
    public double ki;
    public double kiCap;
    public double kd;


    public PIDCoefficients(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kiCap = 0.25;

    }

    public PIDCoefficients(double kp, double ki, double kiCap, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kiCap = kiCap;
    }

}
