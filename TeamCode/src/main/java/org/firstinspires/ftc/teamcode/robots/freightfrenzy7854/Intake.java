package org.firstinspires.ftc.teamcode.robots.freightfrenzy7854;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    DcMotor frontIntakeMotor;
    DcMotor backIntakeMotor;
    Servo intakeFlap;

    double flapStartPosition;

    public Intake(HardwareMap hardwareMap){
        frontIntakeMotor = hardwareMap.get(DcMotorEx.class, "frontMotorIntake");
        backIntakeMotor = hardwareMap.get(DcMotorEx.class, "backMotorIntake");
        intakeFlap = hardwareMap.get(Servo.class, "intakeFlapServo");

        frontIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        backIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeFlap.setDirection(Servo.Direction.FORWARD);

        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flapStartPosition = intakeFlap.getPosition();
    }

    public void forward() {
        frontIntakeMotor.setPower(1.0);
        backIntakeMotor.setPower(1.0);
    }

    public void reverse() {
        frontIntakeMotor.setPower(-1.0);
        backIntakeMotor.setPower(-1.0);
    }

    public void flapOpen() {
        intakeFlap.setPosition(flapStartPosition - 0.8);
    }

    public void flapClose() {
        intakeFlap.setPosition(flapStartPosition);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Front Intake Motor Power", frontIntakeMotor.getPower());
        telemetry.addData("Back Intake Motor Power", backIntakeMotor.getPower());
        telemetry.addData("Flap Servo Position", intakeFlap.getPosition());
    }
}