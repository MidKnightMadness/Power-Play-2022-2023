package org.firstinspires.ftc.teamcode.robots.ultimategoal7854;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robots.freightfrenzy7854.Chassis;
import org.firstinspires.ftc.teamcode.robots.freightfrenzy7854.Intake;

public class UltimateGoal7854 extends OpMode {

    Chassis chassis;
    Outtake outtake;

    private boolean lastPressedOuttake = false;
    private boolean outtakeToggle = false;

    @Override
    public void init() {
        chassis = new Chassis(hardwareMap);
        outtake = new Outtake(hardwareMap);
    }

    public void init_loop() {

    }

    public void start() {

    }

    @Override
    public void loop() {
        // DRIVE
        chassis.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_y);

        // OUTTAKE
        //outtake spinny thingy (toggle)
        if (gamepad2.y && !lastPressedOuttake) {
            outtakeToggle = !outtakeToggle;
        }
        if (outtakeToggle) {
            outtake.start();
        } else {
            outtake.stop();
        }
        lastPressedOuttake = gamepad2.y;

        //outtake spinny thingy (hold)
        if(gamepad2.left_trigger > 0) {
            outtake.setVelocity( (int)(gamepad2.left_trigger * 1800) );
        }

        //outtake feeder
        if (gamepad2.x) {
            outtake.feed();
        }


        // TELEMETRY
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        chassis.telemetry(telemetry);
        outtake.telemetry(telemetry);
        telemetry.update();
    }
}
