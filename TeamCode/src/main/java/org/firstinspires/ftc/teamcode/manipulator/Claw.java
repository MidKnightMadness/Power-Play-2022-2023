package org.firstinspires.ftc.teamcode.manipulator;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Timer;

/*
 * (Expansion Hub)
 * Servos:
 * 0    Claw    Servo
 * 1    CP      CRServo
 * 2    CP2     CRServo
 */

public class Claw {
    public Timer calibrationTimer;

    public Servo servo;
    public Servo rotationServo;
    public Servo rotationServo2;

    double OPEN = 0.425;
    double CLOSED = 0.750;

    public Claw(HardwareMap hardwareMap) {
        calibrationTimer = new Timer();

        servo = hardwareMap.get(Servo.class, "Claw");

        rotationServo = hardwareMap.get(Servo.class, "CP");
        rotationServo.setDirection(Servo.Direction.FORWARD);

        rotationServo2 = hardwareMap.get(Servo.class, "CP2");
        rotationServo2.setDirection(Servo.Direction.REVERSE);

        servo.resetDeviceConfigurationForOpMode();
        rotationServo.resetDeviceConfigurationForOpMode();
        rotationServo2.resetDeviceConfigurationForOpMode();

    }

    public void openClaw() {
        servo.setPosition(OPEN);
    }
    
    public void closeClaw() {
        servo.setPosition(CLOSED);
    }

    public void rotateClaw(double power) {
        rotationServo.setPosition(power);
//        rotationServo2.setPower(power);
    }

    public void waitForOpenClaw() {
        this.openClaw();
        while (servo.getPosition() < servo.MAX_POSITION) { }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("\nCLAW");
        telemetry.addLine(String.format("Claw Servo Position: %f", servo.getPosition()));
        telemetry.addLine(String.format("Claw Rotation Servo Power: %f %f", rotationServo.getPosition(), rotationServo2.getPosition()));
        telemetry.addLine(String.format("Claw Rotation Servo Direction: %s %s", rotationServo.getDirection(), rotationServo2.getDirection()));
    }

}
