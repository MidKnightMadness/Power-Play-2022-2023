package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.common.Timer;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends OpMode implements cameraInfo
{
    int[] signalFinds = new int[] {0, 0, 0};
    int mostRecentDetection = 0;

    AprilTagDetection tagOfInterest = null;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Timer coneTimer;

    MecanumDrive mecanum;
    Odometry odometry;

    @Override
    public void init() {
        coneTimer = new Timer();
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
        coneTimer.getTime();

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == 1 || tag.id == 2 || tag.id == 3)
                {
                    signalFinds[tag.id - 1] += 1;
                    mostRecentDetection = tag.id;
                    tagOfInterest = tag;
                    tagFound = true;
                }
            }

            if (tagFound)
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

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine("-------------------------");
        telemetry.addLine("Found tag " + detection.id + " in " + (coneTimer.getDeltaTime()) + " seconds");
        telemetry.addLine("-------------------------");
    }
}

