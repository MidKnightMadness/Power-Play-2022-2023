package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class Motors extends LinearOpMode {
    Servo testServo;
    DcMotorEx dcMotor;

    boolean servoisActive;
    boolean dcIsActive;

    void HandleInputs() {
        if (this.gamepad1.a) {
            if (servoisActive) {

            }
        }

        if (this.gamepad1.b) {
            dcIsActive = !dcIsActive;
        }
    }
    public void runOpMode(){
        testServo = hardwareMap.get(Servo.class, "servoTest");
        dcMotor = hardwareMap.get(DcMotorEx.class, "dcMotorTest");

        telemetry.addData("Controls", "Use Controls left stick y to turn servo" +
                "Press A to toggle");

        telemetry.addData("Controls", "Use Controls left stick x to turn motor" +
                "Press B to toggle");

        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            HandleInputs();
            if (dcIsActive) {
                dcMotor.setPower(this.gamepad1.left_stick_x);
            }
            else {
                dcMotor.setPower(0);
            }

            if (servoisActive) {
                testServo.setPosition((gamepad1.left_stick_y + 1.0) / 2.0);
            }

            if (servoisActive) {
                telemetry.addData("Servo position: ", testServo.getPosition());
            }
            else {
                telemetry.addData("Servo Inactive", "");
            }

            if (dcIsActive) {
                telemetry.addData("Motor Power: ", dcMotor.getPower());
            }
            else {
                telemetry.addData("Motor is inactive", "");
            }


            telemetry.update();
        }
    }
}
