package org.firstinspires.ftc.teamcode.Robots.FreightFrenzy15385;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    private DcMotorEx liftMotor;
    private Servo liftServo;
    private int motorStartPosition;

    public Lift(HardwareMap hardwareMap) {
        liftServo = hardwareMap.get(Servo.class, "lift_servo");
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift_motor");
        motorStartPosition = liftMotor.getCurrentPosition();
    }

    public void lift() {
        liftMotor.setTargetPosition(1000 + motorStartPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    public void lower() {
        liftMotor.setTargetPosition(motorStartPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    public void open() {
        liftServo.setPosition(100);
    }

    public void close() {
        liftServo.setPosition(0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Motor Current", liftMotor.getCurrentPosition());
        telemetry.addData("Lift Motor Target", liftMotor.getTargetPosition());
        telemetry.addData("Lift Servo Position", liftServo.getPosition());
    }
}
