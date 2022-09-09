package org.firstinspires.ftc.teamcode.practicerobots.FreightFrenzy15385_WestCoast;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Freight Frenzy Mayhem West Coast TeleOp", group="Practice Robots")
public class FreightFrenzy15385_WestCoast extends OpMode {

    WestCoastDrive westcoast;

    @Override
    public void init() {
        westcoast = new WestCoastDrive(hardwareMap);
    }

    public void init_loop() {

    }

    public void start() {

    }

    @Override
    public void loop() {
        westcoast.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        westcoast.telemetry(telemetry);
    }
}
