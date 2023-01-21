package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "Test")
@Disabled
@Deprecated
public class TestDCMotor extends LinearOpMode {
    DcMotorEx dcMotor;

    public void runOpMode() {
        telemetry.addData("Controls", "Left stick X");
        telemetry.update();
        dcMotor = hardwareMap.get(DcMotorEx.class, "SSM");

        waitForStart();

        while (opModeIsActive()) {
            dcMotor.setPower(gamepad1.right_stick_y);
//            if (gamepad1.a) {
//
//            }
//            else {
//                dcMotor.setPower(0);
//            }
            telemetry.addData("a", gamepad1.right_stick_x);
            telemetry.update();
        }

    }
}
