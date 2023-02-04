package org.firstinspires.ftc.teamcode.highlevel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousDrive;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

import java.util.*;

@TeleOp(name = "Spline Tester")
public class TestSpline extends OpMode {
    public static SplinePath splinePath;
    public static double [][] finalPath;
    public static ArrayList <Double> xRoots = new ArrayList <Double> ();
    public static ArrayList <Double> yRoots = new ArrayList <Double> ();
    public static double [][] roots = {
            {0.0, 0.0},
            {0.0, 23.5}
    };

    public static double [] targetCoords = {0.0, 0.0};
    Odometry odometry;
    AutonomousDrive PIDdriver;
    org.firstinspires.ftc.teamcode.common.Timer timer;


    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, Math.PI / 2, new Vector2(0.0, 0.0));
        PIDdriver = new AutonomousDrive(hardwareMap);

        splinePath = new SplinePath(roots);
        finalPath = splinePath.generatePath(0.01, 0.0, 1.0);

        timer = new org.firstinspires.ftc.teamcode.common.Timer();
    }

    @Override
    public void loop() {
        targetCoords = splinePath.update(- gamepad1.left_stick_y, finalPath);

        PIDdriver.setTargetState(telemetry, new Vector2(odometry.getXCoordinate(), odometry.getYCoordinate()),
                new Vector2(targetCoords[0], targetCoords[1]), odometry.getRotationRadians(), odometry.getRotationRadians(),
                timer.getDeltaTime());
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
