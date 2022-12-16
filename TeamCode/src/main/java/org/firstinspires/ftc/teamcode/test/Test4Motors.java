package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Test Wheel Motors", group = "Test")
public class Test4Motors extends OpMode {
    public DcMotorEx FRMotor;
    public DcMotorEx FLMotor;
    public DcMotorEx BRMotor;
    public DcMotorEx BLMotor;

    @Override
    public void init() {
//         Connect Motors
        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");

        // Set Directions
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set Motor Mode
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Zero Power Behavior
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stops Motors on INIT
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);

    }

    @Override
    public void loop() {
        telemetry.addLine("PRESS X Y A B FOR 4 MOTORS");
        telemetry.addLine("FR Motor: X or SQUARE");
        telemetry.addLine("FL Motor: Y or TRIANGLE");
        telemetry.addLine("BR Motor: A or CROSS");
        telemetry.addLine("BL Motor: B or CIRCLE");

        if (gamepad1.x || gamepad1.square) {
            FRMotor.setPower(1);
        } else {
            FRMotor.setPower(0);
        }

        if (gamepad1.y || gamepad1.triangle) {
            FLMotor.setPower(1);
        } else {
            FLMotor.setPower(0);
        }

        if (gamepad1.a || gamepad1.cross) {
            BRMotor.setPower(1);
        } else {
            BRMotor.setPower(0);
        }

        if (gamepad1.b || gamepad1.circle) {
            BLMotor.setPower(1);
        } else {
            BLMotor.setPower(0);
        }
    }
}
