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

// Encoders, Motors
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "main")
public class TeleOp1 extends OpMode {
    MecanumDrive driver;

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

        telemetry.addLine(String.format("\ndrive vector:\t{%f, %f, %f, %f}", drive[0], drive[1], drive[2], drive[3]));




//        telemetry.addData("drive vector reference:\t", drive);
//
//        telemetry.addData("\nController left stick x", gamepad1.left_stick_x);
//        telemetry.addData("Controller left stick y", gamepad1.left_stick_y);
//        telemetry.addData("Controller right stick x", gamepad1.right_stick_x);
//
//
//        telemetry.addData("\nleft front:\t", drive[0]);
//        telemetry.addData("\nright front:\t", drive[1]);
//        telemetry.addData("\nleft back:\t", drive[2]);
//        telemetry.addData("\nright back:\t", drive[3]);


        telemetry.addData("\nFront Left output:", driver.FLMotor.getPower());
        telemetry.addData("Front right output:", driver.FLMotor.getPower());
        telemetry.addData("Rear left output:", driver.FLMotor.getPower());
        telemetry.addData("Rear right output:", driver.FLMotor.getPower());

        telemetry.update();
    }
}