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
    public CRServo rotationServo;


    private ServoController servoController;
    private ServoController rotationServoController;

    private double closedPositionTicks = 0;
    private double openPositionTicks = 0;

    private double downPosition = 0;
    private double upPosition = 0;

    public Claw(HardwareMap hardwareMap) {
        calibrationTimer = new Timer();

//        servo = hardwareMap.get(CRServo.class, "claw");
         servo = hardwareMap.get(Servo.class, "claw");
         rotationServo = hardwareMap.get(CRServo.class, "claw pivot");

        servoController = servo.getController();
        rotationServoController = rotationServo.getController();

        servo.resetDeviceConfigurationForOpMode();
        rotationServo.resetDeviceConfigurationForOpMode();

        // Assumes starting closed
        closedPositionTicks = servo.getPosition() + 0.2;
        openPositionTicks = servo.MAX_POSITION - 0.2;

//        servo.scaleRange(closedPositionTicks, openPositionTicks);
    }

    public void openClaw() {
        servo.setPosition(0);
    }
    
    public void closeClaw() {
        servo.setPosition(1);
    }

    public void waitForOpenClaw() {
        this.openClaw();
        while (servo.getPosition() < servo.MAX_POSITION) {
            continue;
        }
    }

    public void pivotBy(double ticks){
//        rotationServo.setPosition(rotationServo.getPosition() + ticks / 2);
        rotationServo.setPower(ticks);
    }

}
