package org.firstinspires.ftc.teamcode.manipulator;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class Claw {

    Servo servo;
    ServoController servoController;

    public Claw(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "claw");
        servoController = servo.getController();
    }

    public void openClaw() {
        servo.setPosition(servo.MAX_POSITION);
    }
    
    public void closeClaw() {
        servo.setPosition(servo.MIN_POSITION);
    }

    public void waitForOpenClaw() {
        this.openClaw();
        while (servo.getPosition() < servo.MAX_POSITION) {
            continue;
        }
    }

}
