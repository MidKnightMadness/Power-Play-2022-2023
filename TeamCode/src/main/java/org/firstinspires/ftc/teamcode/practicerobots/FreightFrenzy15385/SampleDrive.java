package org.firstinspires.ftc.teamcode.practicerobots.FreightFrenzy15385;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SampleDrive {
    DcMotorEx FRMotor;
    DcMotorEx FLMotor;
    DcMotorEx BRMotor;
    DcMotorEx BLMotor;

    public SampleDrive(HardwareMap hardwareMap) {
        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");

        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);

        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);
    }

    public void drive(double x, double y, double rotation) {
        FRMotor.setPower(-x - y - rotation);
        FLMotor.setPower(-x + y - rotation);
        BRMotor.setPower(-x + y + rotation);
        BLMotor.setPower(-x - y + rotation);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("FR Motor Position", FRMotor.getCurrentPosition());
        telemetry.addData("FL Motor Position", FLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Position", BRMotor.getCurrentPosition());
        telemetry.addData("BL Motor Position", BLMotor.getCurrentPosition());
    }
}
