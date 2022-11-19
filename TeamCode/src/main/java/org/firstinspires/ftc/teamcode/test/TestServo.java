package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "test")
public class TestServo extends LinearOpMode {
    private Servo testServo;

    public void runOpMode(){
        testServo = hardwareMap.get(Servo.class, "servoTest");
        telemetry.addData("Controls", "Use Controls left stick to turn servo" );
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            testServo.setPosition((gamepad1.left_stick_y + 1.0) / 2.0);
            telemetry.addData("Servo position: ", testServo.getPosition());
            telemetry.update();
        }
    }
}