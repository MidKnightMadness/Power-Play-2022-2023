package org.firstinspires.ftc.teamcode.highlevel;

import static org.firstinspires.ftc.teamcode.highlevel.Master.drive;
import static org.firstinspires.ftc.teamcode.highlevel.Master.initEverything;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.drivetrain.*;

// Encoders, Motors
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TeleOp1 extends OpMode {

    @Override
    public void init() {
        initEverything();
    }

    @Override
    public void loop() {

        drive.vectorDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);

        telemetry.addLine(String.format("∆x: [%3.1f%, %3.1f\nrotation: %3.1f", gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
        telemetry.update();
    }
}
