package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;

@TeleOp(name="Main")
public class MainTeleOp extends OpMode {
    MecanumDrive mecanum;

    @Override
    public void init() {
        mecanum = new MecanumDrive(hardwareMap);
    }

    public void init_loop() {

    }

    public void start()
    {

    }

    @Override
    public void loop() {
        //mecanum.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_y);
        mecanum.VectorDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_y, telemetry);
        mecanum.telemetry(telemetry);
    }
}
