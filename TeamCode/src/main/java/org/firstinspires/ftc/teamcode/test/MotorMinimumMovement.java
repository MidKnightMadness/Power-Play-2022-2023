package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorMinimumMovement extends OpMode {
    public DcMotorEx FRMotor;
    public DcMotorEx FLMotor;
    public DcMotorEx BRMotor;
    public DcMotorEx BLMotor;

    @Override
    public void init() {
        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");

        // Set Directions
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set Motor Mode
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    double powerLevel = 0.0;

    double min = 0.01;
    double max = 0.11;

    @Override
    public void loop() {
        powerLevel = min + (this.gamepad1.left_stick_x + 1) / 2.0 * 0.1;
        telemetry.addData("Powerlevel", powerLevel);
        BLMotor.setPower(powerLevel);
        FLMotor.setPower(powerLevel);
        FRMotor.setPower(powerLevel);
        BRMotor.setPower(powerLevel);

    }

}
