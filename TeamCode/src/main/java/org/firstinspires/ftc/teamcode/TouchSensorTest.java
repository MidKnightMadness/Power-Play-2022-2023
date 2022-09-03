package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(group="TeleOp")
public class TouchSensorTest extends LinearOpMode {
    Servo testServo;
    com.qualcomm.robotcore.hardware.TouchSensor testTouchSensor;

    public void runOpMode() {
        testServo = hardwareMap.servo.get("servoTest");
        testTouchSensor = hardwareMap.touchSensor.get("touchSensor");

        waitForStart();


        while (opModeIsActive()) {
            if (testTouchSensor.isPressed()) {
                testServo.setPosition(0);
            } else {
                testServo.setPosition(1);
            }

            telemetry.addData("Touch sensor", testTouchSensor.isPressed());
            telemetry.update();
        }

    }


}