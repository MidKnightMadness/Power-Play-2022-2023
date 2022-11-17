package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.highlevel.Master.telemetry;


import org.firstinspires.ftc.teamcode.drivetrain.Vector;
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


    public static double [] drive = {0.0, 0.0, 0.0, 0.0};
    public static double [] translation = {0.0, 0.0, 0.0, 0.0};
    public static double [] rotation = {0.0, 0.0, 0.0, 0.0};

    public static double [] BACKWARDS = {-1.0, 1.0, -1.0, 1.0};

    public static double [] RIGHT = {1.0, 1.0, -1.0, -1.0};

    public static final double [] TURN_RIGHT = {-1.0, -1.0, -1.0, -1.0};

    @Override
    public void init() {
        telemetry.addData("\"RIGHT\"\t", this.RIGHT);
        telemetry.addData("\"BACKWARDS\"\t", this.BACKWARDS);
        telemetry.addData("\"TURN_RIGHT\"\t", this.TURN_RIGHT);
        telemetry.addData("drive vector\t", this.drive);

        telemetry.addLine(String.format("\nleft front:\t%f", this.RIGHT[0]));
        telemetry.addLine(String.format("\nright front:\t%f", this.RIGHT[1]));
        telemetry.addLine(String.format("\nleft back:\t%f", this.RIGHT[2]));
        telemetry.addLine(String.format("\nright back:\t%f", this.RIGHT[3]));
        telemetry.update();
    }

    @Override
    public void loop() {
        this.drive = vectorDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, this.BACKWARDS, this.RIGHT, this.TURN_RIGHT, this.drive, this.translation, this.rotation);

        telemetry.addData("drive vector reference:\t", this.drive);

        telemetry.addData("\nController left stick x", gamepad1.left_stick_x);
        telemetry.addData("Controller left stick y", gamepad1.left_stick_y);
        telemetry.addData("Controller right stick x", gamepad1.right_stick_x);


        telemetry.addData("\nleft front:\t", this.drive[0]);
        telemetry.addData("\nright front:\t", this.drive[1]);
        telemetry.addData("\nleft back:\t", this.drive[2]);
        telemetry.addData("\nright back:\t", this.drive[3]);


//        telemetry.addData("\nFront Left output:", this.FLMotor.getPower());
//        telemetry.addData("Front right output:", this.FLMotor.getPower());
//        telemetry.addData("Rear left output:", this.FLMotor.getPower());
//        telemetry.addData("Rear right output:", this.FLMotor.getPower());

        telemetry.update();
    }

    public static double [] vectorDrive(double x, double y, double rotate, double [] BACKWARDS, double [] RIGHT, double [] TURN_RIGHT, double [] drive, double [] translation, double [] rotation) {

//        translation = Vector.add(Vector.multiply(x, RIGHT), Vector.multiply(y, BACKWARDS));
//        rotation = Vector.multiply(rotate, TURN_RIGHT);
//        TeleOp1.drive = Vector.multiply(0.5, translation); // temporary ig
//
//
//        Vector.multiply(1/Math.max(drive[0], Math.max(drive[1], Math.max(drive[2], drive[3]))), drive);

        return drive;

//        FLMotor.setPower(this.drive[0]);
//        FRMotor.setPower(this.drive[1]);
//        BLMotor.setPower(this.drive[2]);
//        BRMotor.setPower(this.drive[3]);
    }
}


