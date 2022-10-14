package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
public class TestDCMotor extends LinearOpMode {
    DcMotorEx dcMotor;

    public void runOpMode() {
        telemetry.addData("Controls", "Left stick X");
        telemetry.update();
        dcMotor = hardwareMap.get(DcMotorEx.class, "dcMotorTest");

        waitForStart();

        while (opModeIsActive()) {
            if (this.gamepad1.a) {
                dcMotor.setPower(this.gamepad1.left_stick_x);
            }
            else {
                dcMotor.setPower(0);
            }
        }

        telemetry.addData("a", this.gamepad1.left_stick_x);
        telemetry.update();
    }
}
