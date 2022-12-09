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
    public Servo rotationServo;
    public Servo rotationServo2;


    private ServoController servoController;
//    private ServoController rotationServoController;
//        private ServoController rotationServoController2;

    double OPEN = 0.425;
    double CLOSED = 0.750;

    private double downPosition = 0;
    private double upPosition = 0;

    public boolean open = false;

    public Claw(HardwareMap hardwareMap) {
        calibrationTimer = new Timer();

         servo = hardwareMap.get(Servo.class, "Claw");

         rotationServo = hardwareMap.get(Servo.class, "CP");
        rotationServo.setDirection(Servo.Direction.FORWARD);

         rotationServo2 = hardwareMap.get(Servo.class, "CP2");
         rotationServo2.setDirection(Servo.Direction.REVERSE);




//        servoController = servo.getController();
//        rotationServoController = rotationServo.getController();
//                rotationServoController2 = rotationServo2.getController();


        servo.resetDeviceConfigurationForOpMode();
        rotationServo.resetDeviceConfigurationForOpMode();
        rotationServo2.resetDeviceConfigurationForOpMode();





    }

    public void openClaw() {
        servo.setPosition(OPEN);
        boolean open = true;
    }
    
    public void closeClaw() {
        servo.setPosition(CLOSED);
        boolean open = false;
    }

    public void waitForOpenClaw() {
        this.openClaw();
        while (servo.getPosition() < servo.MAX_POSITION) { }
    }

    public void pivotBy(double ticks){
        rotationServo.setPosition(rotationServo.getPosition() + ticks / 2);

        rotationServo2.setPosition(rotationServo.getPosition() + ticks / 2);

    }

    public static final double WRIST_RATIO = 0.5 / Math.PI;  // Ticks per radian
    public void pivotTo(double angle){ // Radians

    }

}
