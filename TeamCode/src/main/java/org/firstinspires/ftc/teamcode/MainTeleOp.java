package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.highlevel.Master.drive;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.Odometry.Odometry;

@TeleOp(name="Main")
public class MainTeleOp extends OpMode {
    MecanumDrive mecanum;
    Odometry odometry;
    LinearSlides lift;
    Turntable turntable;

    private boolean lastPressedLiftMotor = false;
    private boolean liftMotorToggle = false;
    private boolean lastPressedDriveMode = false;
    private boolean driveModeToggle = false;

    @Override
    public void init() {
        mecanum = new MecanumDrive(hardwareMap);
//        odometry = new Odometry(hardwareMap);
//        lift = new LinearSlides(hardwareMap);
//        turntable = new Turntable(hardwareMap);

    }

    @Override
    public void loop() {
        // DRIVER ASSIST

        if (gamepad1.right_bumper && !lastPressedDriveMode) {
            driveModeToggle = !driveModeToggle;
        }
        if (driveModeToggle) {
            mecanum.fieldOrientatedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);
        } else {
            mecanum.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x/2); // normal drive
        }
        lastPressedDriveMode = gamepad1.right_bumper;

        //mecanum.vectorDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_y, telemetry);

        // LIFT (DOUBLE REVERSE 4 BAR)
        /*if (gamepad2.dpad_up && !lastPressedLiftMotor) {
            liftMotorToggle = !liftMotorToggle;
        }
        if (liftMotorToggle) {
            lift.raise();
        } else {
            lift.lower();
        }
        lastPressedLiftMotor = gamepad2.dpad_up;*/

        // TURNTABLE
        /*if (gamepad2.right_stick_x != 0) {
            turntable.turn(gamepad2.right_stick_x);
        }*/


//        odometry.updateTime();
//        odometry.updatePosition();
//
//        // TELEMETRY
//        odometry.telemetry(telemetry);
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        mecanum.telemetry(telemetry);
        telemetry.addLine("Drive Mode: " + driveModeToggle);
        telemetry.update();
    }
}
