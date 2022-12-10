package org.firstinspires.ftc.teamcode.manipulator;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Timer;

public class Claw {
    public Timer calibrationTimer;

    public Servo servo;
    public CRServo rotationServo;
    public CRServo rotationServo2;

    double OPEN = 0.425;
    double CLOSED = 0.750;

    public Claw(HardwareMap hardwareMap) {
        calibrationTimer = new Timer();

         servo = hardwareMap.get(Servo.class, "Claw");

         rotationServo = hardwareMap.get(CRServo.class, "CP");
         rotationServo.setDirection(CRServo.Direction.FORWARD);

         rotationServo2 = hardwareMap.get(CRServo.class, "CP2");
         rotationServo2.setDirection(CRServo.Direction.REVERSE);




//        servoController = servo.getController();
//        rotationServoController = rotationServo.getController();
//                rotationServoController2 = rotationServo2.getController();


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
        rotationServo.setPower(power);
    }

    public void waitForOpenClaw() {
        this.openClaw();
        while (servo.getPosition() < servo.MAX_POSITION) { }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("\nCLAW");
        telemetry.addLine(String.format("Claw Servo Position: %f", servo.getPosition()));
        telemetry.addLine(String.format("Claw Rotation Servo Power: %f %f", rotationServo.getPower(), rotationServo2.getPower()));
    }

}
