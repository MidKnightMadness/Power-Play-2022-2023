package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;


@TeleOp(name = "Test Odometry", group = "Test")
public class OdometryTest extends OpMode{

    Odometry odometry;
    MecanumDrive mecanum;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, new Vector2(0.0, 0.0));
        mecanum = new MecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.x || gamepad1.square) {
            odometry.resetEncoders();
        }

        mecanum.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                (gamepad1.right_stick_x + gamepad2.left_stick_x) * 0.5); // normal drive

        odometry.updateTime();
        odometry.updatePosition();

        telemetry.addLine("PRESS X OR SQUARE to RESET ENCODERS");
        odometry.telemetry(telemetry);
        telemetry.update();

//        try {
//            Thread.sleep(sleepTime);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }

    }

}

