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
 * 1    CP      Servo
 * 2    CP2     Servo // Deprecated
 */

public class Claw {
    public Servo servo;
    public Servo rotationServo;
//    public Servo rotationServo2;

    double OPEN = 0.425;
    double CLOSED = 0.750;

    public Claw(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "Claw");

        rotationServo = hardwareMap.get(Servo.class, "CP");
        rotationServo.resetDeviceConfigurationForOpMode();

        servo.scaleRange(0.0, 1.0);
        rotationServo.scaleRange(0.0, 1.0);

//        rotationServo.setPosition(0.0);

//        rotationServo2 = hardwareMap.get(Servo.class, "CP2");
//        rotationServo2.setDirection(Servo.Direction.REVERSE);

    }

    public void openClaw() {
        servo.setPosition(OPEN);
    }
    
    public void closeClaw() {
        servo.setPosition(CLOSED);
    }

    public void rotateClaw(double power) {
//        rotationServo.setPosition((rotationServo.getPosition()+power+1)%2-1);
        rotationServo.setPosition(power);
    }

    public void waitForOpenClaw() {
        this.openClaw();
        while (servo.getPosition() < servo.MAX_POSITION) { }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addLine("CLAW");
        telemetry.addLine(String.format("Claw Servo Position: %f", servo.getPosition()));
//        telemetry.addLine(String.format("Claw Rotation Servo Power: %f %f", rotationServo.getPosition(), rotationServo2.getPosition()));
//        telemetry.addLine(String.format("Claw Rotation Servo Direction: %s %s", rotationServo.getDirection(), rotationServo2.getDirection()));
    }

}
