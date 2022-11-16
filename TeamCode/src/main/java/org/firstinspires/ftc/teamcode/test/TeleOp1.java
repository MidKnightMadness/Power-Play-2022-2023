package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.highlevel.Master.telemetry;


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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "main")
public class TeleOp1 extends OpMode {
    MecanumDrive drive;
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        telemetry.addData("\"RIGHT\" reference\t\t", drive.RIGHT);
        telemetry.addData("\"BACKWARDS\" reference:\t", drive.BACKWARDS);
        telemetry.addData("\"TURN_RIGHT\" reference:\t", drive.TURN_RIGHT);
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.vectorDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);


        telemetry.addData("Controller left stick x", gamepad1.left_stick_x);
        telemetry.addData("Controller left stick y", gamepad1.left_stick_y);
        telemetry.addData("Controller right stick x", gamepad1.right_stick_x);

        telemetry.addData("\nleft front", drive.drive[0]);
        telemetry.addData("right front", drive.drive[1]);
        telemetry.addData("left rear", drive.drive[2]);
        telemetry.addData("right rear", drive.drive[3]);


        telemetry.addData("\nFront Left output:", drive.FLMotor.getPower());
        telemetry.addData("Front right output:", drive.FLMotor.getPower());
        telemetry.addData("Rear left output:", drive.FLMotor.getPower());
        telemetry.addData("Rear right output:", drive.FLMotor.getPower());
        telemetry.update();

        telemetry.update();
    }
}
