package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.highlevel.Master.hardwaremap;


import org.firstinspires.ftc.teamcode.drivetrain.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.highlevel.Master;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.drivetrain.*;

// Encoders, Motors
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "main")
public class TeleOp1 extends OpMode {
    MecanumDrive drive;
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        telemetry.addData("\"RIGHT\" reference: ", drive.RIGHT.getVector());
        telemetry.addData("\"BACKWARDS\" reference:\t\t[]", drive.BACKWARDS.getVector());
        telemetry.addData("\"TURN_RIGHT\" reference: \t[]", drive.TURN_RIGHT.getVector());
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.vectorDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addLine(String.format("deltaX: %3.1f, %3.1f%nrotation: %3.1f", gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));

        telemetry.addData("\nFront Left output:", drive.FLMotor.getPower());
        telemetry.addData("Front right output:", drive.FLMotor.getPower());
        telemetry.addData("Rear left output:", drive.FLMotor.getPower());
        telemetry.addData("Rear right output:", drive.FLMotor.getPower());
        telemetry.update();

        telemetry.update();
    }
}
