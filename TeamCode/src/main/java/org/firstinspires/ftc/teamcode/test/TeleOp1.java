package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.highlevel.Master.telemetry;


//import org.firstinspires.ftc.teamcode.drivetrain.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.highlevel.Master;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.drivetrain.*;
import org.firstinspires.ftc.teamcode.odometry.TestingOdometryAlgorithm;

// Encoders, Motors
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "main")
public class TeleOp1 extends OpMode {
    MecanumDrive driver;
    TestingOdometryAlgorithm odometry;

    public static double [] currentPosition = {0.0, 0.0};

    public static double [] drive = {0.0, 0.0, 0.0, 0.0};

    public static double [] BACKWARDS = {-1.0, 1.0, -1.0, 1.0};

    public static double [] RIGHT = {1.0, 1.0, -1.0, -1.0};

    public static final double [] TURN_RIGHT = {-1.0, -1.0, -1.0, -1.0};

    @Override
    public void init() {

        for(int i = 0; i < 4; i++){
            drive[i] = 0.0;
        }

        driver = new MecanumDrive(hardwareMap);
        odometry = new TestingOdometryAlgorithm(hardwareMap);

        telemetry.addData("\"RIGHT\"\t", this.RIGHT);
        telemetry.addData("\"BACKWARDS\"\t", this.BACKWARDS);
        telemetry.addData("\"TURN_RIGHT\"\t", this.TURN_RIGHT);
        telemetry.addData("\ndrive vector\t", drive);

        telemetry.addLine(String.format("\nleft front:\t%f", this.RIGHT[0]));
        telemetry.addLine(String.format("\nright front:\t%f", this.RIGHT[1]));
        telemetry.addLine(String.format("\nleft back:\t%f", this.RIGHT[2]));
        telemetry.addLine(String.format("\nright back:\t%f", this.RIGHT[3]));
        telemetry.update();
    }

    @Override
    public void loop() {
        odometry.updateOrientationAndLocation();

        drive[0] = gamepad1.left_stick_y * BACKWARDS[0] + gamepad1.left_stick_x * RIGHT[0];
        drive[1] = gamepad1.left_stick_y * BACKWARDS[1] + gamepad1.left_stick_x * RIGHT[1];
        drive[2] = gamepad1.left_stick_y * BACKWARDS[2] + gamepad1.left_stick_x * RIGHT[2];
        drive[3] = gamepad1.left_stick_y * BACKWARDS[3] + gamepad1.left_stick_x * RIGHT[3];
        
        for(int i = 0; i < 4; i++){
            drive[i] = gamepad1.left_stick_y * BACKWARDS[i];
        }

        for(int i = 0; i < 4; i++){
            drive[i] += gamepad1.left_stick_x * RIGHT[i];
        }

        for(int i = 0; i < 4; i++){
            drive[i] += gamepad1.right_stick_x * TURN_RIGHT[i];
        }

        for(int i = 0; i < 4; i++){
            drive[i] /= Math.max(drive[0], Math.max(drive[1], Math.max(drive[2], drive[3])));
        }


        driver.FLMotor.setPower(drive[0]);
        driver.FRMotor.setPower(drive[1]);
        driver.BLMotor.setPower(drive[2]);
        driver.BRMotor.setPower(drive[3]);

        telemetry.addLine(String.format("\ndrive vector:\t{%3.2f, %3.2f, %3.2f, %3.2f}", drive[0], drive[1], drive[2], drive[3]));

        telemetry.addLine(String.format("\nposition vector:\t{%4.2f, %4.2f}", currentPosition[0], currentPosition[1]));
        telemetry.addData("\nangle vector", odometry.orientationAngle * 180 / Math.PI);

        telemetry.addData("\nFront Left output:", driver.FLMotor.getPower());
        telemetry.addData("Front right output:", driver.FLMotor.getPower());
        telemetry.addData("Rear left output:", driver.FLMotor.getPower());
        telemetry.addData("Rear right output:", driver.FLMotor.getPower());

        telemetry.update();
    }
}