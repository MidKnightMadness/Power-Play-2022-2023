package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.highlevel.Master.odometryAlg;

/*
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.common.Timer;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

interface cameraInfo {
    final double FEET_PER_METER = 3.28084;

    final double fx = 578.272;
    final double fy = 578.272;
    final double cx = 402.145;
    final double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

}

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends OpMode implements cameraInfo
{
    int[] signalFinds = new int[] {0, 0, 0};

    AprilTagDetection tagOfInterest = null;

    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Timer timer;
    MecanumDrive mecanum;
    Odometry odometry;

    @Override
    public void init() {
        timer = new Timer();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        telemetry.setAutoClear(false);
        mecanum = new MecanumDrive(hardwareMap);
        odometry = new Odometry(hardwareMap);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("ERROR", "Error code" + errorCode);
            }
        });
    }

    @Override
    public void init_loop() {
        timer.getTime();

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == 1 || tag.id == 2 || tag.id == 3)
                {
                    signalFinds[tag.id - 1] += 1;
                    telemetry.addLine("-------------------------");
                    telemetry.addLine("Found tag " + tag.id + " in " + (timer.getDeltaTime()) + " seconds");
                    telemetry.addLine("-------------------------");
                    tagOfInterest = tag;
                    tagFound = true;
                }
            }

            if(tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        while (Math.abs(odometry.getXCoordinate() - 23.5) < .2 && Math.abs(odometry.getYCoordinate() - 23.5) < .2) {
            //mecanum.driveTo(odometry.getXCoordinate(), odometry.getYCoordinate(),23.5, 23.5);
            odometryAlg.updateOrientationAndLocation();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}

*/