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

    public static double [] drive = new double [4];

    public static double [] translation = {0.0, 0.0, 0.0, 0.0};
    public static double [] translation1 = {0.0, 0.0, 0.0, 0.0};
    public static double [] rotation = {0.0, 0.0, 0.0, 0.0};

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

        // Temporarily using translation and rotation as intermediary
        translation = Vector.multiply(gamepad1.left_stick_y, BACKWARDS);
        drive = Vector.equalTo(drive, translation); // IDK if this works lmao

        translation1 = Vector.multiply(gamepad1.left_stick_x, RIGHT);
        Vector.equalTo(translation, translation1); // Frees up auxillary reference
        drive = Vector.equalTo(drive, Vector.add(drive, translation)); // Frees up auxillary reference

        rotation = Vector.multiply(gamepad1.right_stick_x, TURN_RIGHT);
        translation1 = Vector.equalTo(translation1, rotation); // Frees up auxillary reference
        drive = Vector.equalTo(drive, Vector.add(drive, translation1));

        // Drive should now be untied from auxillary reference
        drive = Vector.equalTo(drive, Vector.multiply(1/Math.max(drive[0], Math.max(drive[1], Math.max(drive[2], drive[3]))), drive));


        telemetry.addLine(String.format("drive vector:\t{%f, %f, %f, %f}", drive[0], drive[1], drive[2], drive[3]));




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


//        telemetry.addData("\nFront Left output:", this.FLMotor.getPower());
//        telemetry.addData("Front right output:", this.FLMotor.getPower());
//        telemetry.addData("Rear left output:", this.FLMotor.getPower());
//        telemetry.addData("Rear right output:", this.FLMotor.getPower());

        telemetry.update();
    }
}