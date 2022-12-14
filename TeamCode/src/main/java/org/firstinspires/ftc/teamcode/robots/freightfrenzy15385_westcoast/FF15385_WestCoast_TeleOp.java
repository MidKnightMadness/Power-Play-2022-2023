package org.firstinspires.ftc.teamcode.robots.freightfrenzy15385_westcoast;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Freight Frenzy Mayhem West Coast TeleOp", group="Practice Robots")
@Disabled
public class FF15385_WestCoast_TeleOp extends OpMode {

    WestCoastDrive westcoast;

    private boolean lastPressedDriveMode = false;
    private boolean driveModeToggle = false;

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
        if (gamepad1.right_bumper && !lastPressedDriveMode) {
            driveModeToggle = !driveModeToggle;
        }
        if (driveModeToggle) {
            westcoast.drive(gamepad1.left_stick_x / 8.0, -gamepad1.left_stick_y / 8.0, gamepad1.right_stick_x / 8.0);
        } else {
            westcoast.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
        lastPressedDriveMode = gamepad1.right_bumper;

        if (driveModeToggle) telemetry.addLine("Slow");
        else telemetry.addLine("Fast");

        westcoast.telemetry(telemetry);
    }
}
