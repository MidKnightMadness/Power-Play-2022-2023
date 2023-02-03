package org.firstinspires.ftc.teamcode.highlevel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

import java.util.*;

@TeleOp(name = "Spline Tester")
public class TestSpline extends OpMode {
    public static SplinePath path;
    public static ArrayList <Double> xRoots = new ArrayList <Double> ();
    public static ArrayList <Double> yRoots = new ArrayList <Double> ();
    public static double [][] roots = {
            {0.0, 0.0},
            {0.0, 23.5}
    };

    MecanumDrive mecanumDrive;
    Odometry odometry;


    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, Math.PI / 2, new Vector2(0.0, 0.0));
        mecanumDrive = new MecanumDrive(hardwareMap);

        path = new SplinePath(roots);
//        SplinePath.interpolate(path)

    }

    @Override
    public void loop() {

    }

//    public double [][] setRoots(){
//
//        while(true){
//
//            // Start Telemetry Block ============================================================================================================================================
////            telemetry.addData("");
//            // End Telemetry Block ============================================================================================================================================
//
//            if(gamepad1.x){
//                break;
//            }
//        }
//
////        return finalPath;
//    }
}
