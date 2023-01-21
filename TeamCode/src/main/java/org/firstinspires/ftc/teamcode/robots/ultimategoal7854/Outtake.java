package org.firstinspires.ftc.teamcode.robots.ultimategoal7854;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    DcMotorEx shooterMotor;
    Servo feedServo;

    public Outtake(HardwareMap hardwareMap){
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        feedServo = hardwareMap.get(Servo.class, "feedServo");

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setVelocity(0);

        feedServo.setDirection(Servo.Direction.FORWARD);
    }

    public void start() {
        shooterMotor.setVelocity(2500);
    }

    public void stop() {
        shooterMotor.setVelocity(0);
    }

    public void setVelocity(int velocity) {
        shooterMotor.setVelocity(velocity);
    }

    public void feed() {
        feedServo.setPosition(0.65);
//        try {
//            wait(2000);
//        } catch (InterruptedException e) {
////            e.printStackTrace();
//        }
    }

    public void reset() {
        feedServo.setPosition(0.35);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Feeding Servo Position", feedServo.getPosition());
        telemetry.addData("Outtake Motor Velocity", shooterMotor.getVelocity());
    }
}
