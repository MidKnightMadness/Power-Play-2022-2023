package org.firstinspires.ftc.teamcode.highlevel;

import static org.firstinspires.ftc.teamcode.highlevel.Master.drive;
import static org.firstinspires.ftc.teamcode.highlevel.Master.hardwaremap;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.drivetrain.*;

// Encoders, Motors
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TeleOp1 extends OpMode {
    public static Gamepad gamepad1;

    Master master;
    @Override
    public void init() {
        master = new Master();
    }

    @Override
    public void loop() {
        gamepad1 = hardwaremap.get(Gamepad.class, "Gamepad 1");

        drive.vectorDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);

        telemetry.addLine(String.format("∆x: [%3.1f%, %3.1f\nrotation: %3.1f", gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
        telemetry.update();
    }
}
