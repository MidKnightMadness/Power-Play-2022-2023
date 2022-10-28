package org.firstinspires.ftc.teamcode.highlevel;

import static org.firstinspires.ftc.teamcode.highlevel.Master.hardwaremap;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.drivetrain.*;

// Encoders, Motors
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TeleOp1 extends OpMode {


    Master master;
    MecanumDrive drive;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);

    }

    @Override
    public void loop() {
        drive.vectorDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);

        telemetry.addLine(String.format("deltaX: [%3.1f, %3.1f%nrotation: %3.1f", gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
        telemetry.update();
    }
}
