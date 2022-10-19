package org.firstinspires.ftc.teamcode.robots.freightfrenzy7854;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FreightFrenzy7854 extends OpMode {

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
        chassis.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_y);


        // INTAKE
        //intake surgical tubing
        if(gamepad2.dpad_up) {
            intake.forward();
        }
        if(gamepad2.dpad_down) {
            intake.reverse();
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
        chassis.telemetry(telemetry);
        intake.telemetry(telemetry);
        telemetry.update();
    }
}