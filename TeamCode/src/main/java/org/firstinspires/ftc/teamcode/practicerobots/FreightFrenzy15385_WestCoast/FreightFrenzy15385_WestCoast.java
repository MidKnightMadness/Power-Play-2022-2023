package org.firstinspires.ftc.teamcode.practicerobots.FreightFrenzy15385_WestCoast;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Freight Frenzy Mayhem West Coast TeleOp", group="Practice Robots")
public class FreightFrenzy15385_WestCoast extends OpMode {
    private DcMotorEx FRMotor;
    private DcMotorEx FLMotor;
    private DcMotorEx BRMotor;
    private DcMotorEx BLMotor;

    @Override
    public void init() {
        // Mecanum wheel motors
        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void init_loop() {

    }

    public void start() {

    }

    @Override
    public void loop() {
        drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry();
    }



    // DRIVE FUNCTION
    public void drive(double x, double y, double rotation) {
        FRMotor.setPower(-x - y - rotation);
        FLMotor.setPower(-x + y - rotation);
        BRMotor.setPower(-x + y + rotation);
        BLMotor.setPower(-x - y + rotation);
    }

    // TELEMETRY
    public void telemetry() {
        telemetry.addData("FR Motor Position", FRMotor.getCurrentPosition());
        telemetry.addData("FL Motor Position", FLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Position", BRMotor.getCurrentPosition());
        telemetry.addData("BL Motor Position", BLMotor.getCurrentPosition());
        telemetry.addData("FR Motor Velocity", FRMotor.getVelocity());
        telemetry.addData("FL Motor Velocity", FLMotor.getVelocity());
        telemetry.addData("BR Motor Velocity", BRMotor.getVelocity());
        telemetry.addData("BL Motor Velocity", BLMotor.getVelocity());
        telemetry.addData("FR Motor Power", FRMotor.getPower());
        telemetry.addData("FL Motor Power", FLMotor.getPower());
        telemetry.addData("BR Motor Power", BRMotor.getPower());
        telemetry.addData("BL Motor Power", BLMotor.getPower());
        telemetry.update();
    }
}
