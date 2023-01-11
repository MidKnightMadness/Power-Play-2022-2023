package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;



@TeleOp(name = "Test Odometry", group = "Test")
public class OdometryTest extends OpMode{

    Odometry odometry;

    @Override
    public void init() {
//        odometry = new Odometry(hardwareMap, new Vector2(0.0, 0.0), 0);
    }

    @Override
    public void loop() {

        if (gamepad1.x || gamepad1.square) {
            odometry.resetEncoders();
        }

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

