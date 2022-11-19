package org.firstinspires.ftc.teamcode.manipulator;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.common.Timer;

public class Claw {
    public Timer calibrationTimer;


    // Alternative
    public Servo servo;

    private ServoController servoController;

    private double closedPositionTicks = 0;
    private double openPositionTicks = 0;

    public Claw(HardwareMap hardwareMap) {
        calibrationTimer = new Timer();

//        servo = hardwareMap.get(CRServo.class, "claw");
         servo = hardwareMap.get(Servo.class, "claw");

        servoController = servo.getController();

        servo.resetDeviceConfigurationForOpMode();

        // Assumes starting closed
        closedPositionTicks = servo.getPosition() + 0.2;
        openPositionTicks = servo.MAX_POSITION - 0.2;

        servo.scaleRange(closedPositionTicks, openPositionTicks);
    }

    public void openClaw() {
        servo.setPosition(openPositionTicks);
    }
    
    public void closeClaw() {
        servo.setPosition(closedPositionTicks);
    }

    public void waitForOpenClaw() {
        this.openClaw();
        while (servo.getPosition() < servo.MAX_POSITION) {
            continue;
        }
    }

}
