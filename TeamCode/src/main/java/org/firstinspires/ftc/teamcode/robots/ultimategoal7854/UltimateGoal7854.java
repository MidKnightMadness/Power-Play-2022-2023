package org.firstinspires.ftc.teamcode.robots.ultimategoal7854;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "2 Player Carnival UG7854", group = "Carnival")
@Disabled
public class UltimateGoal7854 extends OpMode {

    Chassis chassis;
    Outtake outtake;

    private boolean lastPressedOuttake = false;
    private boolean outtakeToggle = false;
    private boolean lastPressedFeed = false;
    private boolean feedToggle = false;
    ElapsedTime t = new ElapsedTime();

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
        chassis.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

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
//        if (gamepad2.x) {
//            outtake.feed();
//        } else {
//            outtake.reset();
//        }

//        if (gamepad2.x && !lastPressedFeed) {
//            feedToggle = !feedToggle;
//            t.reset();
//        }
//        if (feedToggle && t.seconds() < 0.25) {
//            outtake.feed();
//        } else {
//            outtake.reset();
//        }
//        lastPressedFeed = gamepad2.x;


        if (gamepad2.x && !lastPressedFeed) {
            feedToggle = !feedToggle;
            t.reset();
        }
        if (feedToggle && t.seconds() < 0.3) {
            outtake.feed();
        } else {
            outtake.reset();
        }
        lastPressedFeed = gamepad2.x;


        // TELEMETRY
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Left Joystick x", gamepad1.left_stick_x);
        telemetry.addData("Left Joystick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Joystick X", gamepad1.right_stick_x);
        telemetry.addData("x (flap)", gamepad1.x);
        telemetry.addData("seconds", t.seconds());
        chassis.telemetry(telemetry);
        outtake.telemetry(telemetry);
        telemetry.update();
    }
}
