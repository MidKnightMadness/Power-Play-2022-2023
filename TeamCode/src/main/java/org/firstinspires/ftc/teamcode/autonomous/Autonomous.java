package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AprilTagDetection.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

interface cameraInfo {
    static final double FEET_PER_METER = 3.28084;

    final double fx = 578.272;
    final double fy = 578.272;
    final double cx = 402.145;
    final double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

}

@TeleOp
public class Autonomous extends OpMode
{
    int signalLocation;

    AprilTagDetection tagOfInterest = null;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}

