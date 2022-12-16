package org.firstinspires.ftc.teamcode.robots.freightfrenzy7854;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "2 Player Carnival FF7854", group = "Carnival")
@Disabled
public class FreightFrenzy7854_2P extends OpMode {

    Chassis chassis;
    Intake intake;

    private boolean lastPressedFlap = false;
    private boolean flapToggle = false;

    @Override
    public void init() {
        chassis = new Chassis(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public void init_loop() {

    }

    public void start()
    {

    }

    @Override
    public void loop() {
        // DRIVE
        chassis.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);


        // INTAKE
        //intake surgical tubing
        if(gamepad2.left_trigger > 0) {
            intake.forward();
        } else if(gamepad2.right_trigger > 0) {
            intake.reverse();
        } else {
            intake.stop();
        }

        //intake flap
        if (gamepad2.x && !lastPressedFlap) {
            flapToggle = !flapToggle;
        }
        if (flapToggle) {
            intake.flapOpen();
        } else {
            intake.flapClose();
        }
        lastPressedFlap = gamepad2.x;


        // TELEMETRY
        telemetry.addData("Left Joystick x", gamepad1.left_stick_x);
        telemetry.addData("Left Joystick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Joystick X", gamepad1.right_stick_x);
        telemetry.addData("dpad up (forward)", gamepad1.dpad_up);
        telemetry.addData("dpad down (reverse)", gamepad1.dpad_down);
        telemetry.addData("x (flap)", gamepad1.x);
        telemetry.addData("last pressed flap", lastPressedFlap);
        telemetry.addData("flap toggle", flapToggle);
        chassis.telemetry(telemetry);
        intake.telemetry(telemetry);
        telemetry.update();
    }
}