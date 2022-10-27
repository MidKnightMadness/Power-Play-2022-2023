package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.highlevel.Master.drive;
import static org.firstinspires.ftc.teamcode.highlevel.Master.initEverything;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.odometry.*;
import org.firstinspires.ftc.teamcode.odometry.TestingOdometryAlgorithm;
import org.firstinspires.ftc.teamcode.highlevel.Master;

@TeleOp(name="Main")
public class MainTeleOp extends OpMode {

    @Override
    public void init() {
        initEverything();
    }

    @Override
    public void loop() {
        // DRIVER ASSIST
        //mecanum.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_y); // normal drive
        //mecanum.vectorDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_y, telemetry);
//        drive.fieldOrientatedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_y, telemetry);

        // LIFT (LINEAR SLIDES)
        /*if (gamepad2.dpad_up && !lastPressedLiftMotor) {
            liftMotorToggle = !liftMotorToggle;
        }
        if (liftMotorToggle) {
            lift.raise(1000);
        } else {
            lift.lower();
        }
        lastPressedLiftMotor = gamepad2.dpad_up;*/

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
//        mecanum.telemetry(telemetry);
        telemetry.update();
    }
}
