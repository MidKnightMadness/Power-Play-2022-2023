package org.firstinspires.ftc.teamcode.manipulator;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    Servo servo;
    int openPos = 1;
    int closedPos = 0;

    public void openClaw() {
        while (servo.getPosition() < openPos) {
            servo.setPosition(1);
        }
    }

    public void closeClaw() {
        
    }
}
