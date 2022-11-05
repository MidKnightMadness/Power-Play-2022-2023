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
import org.firstinspires.ftc.teamcode.highlevel.Master;
import org.firstinspires.ftc.teamcode.highlevel.fieldData;
import org.firstinspires.ftc.teamcode.common.Timer;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends OpMode implements cameraInfo, fieldData
{
    public int startingPos = 0;  // 0: A2, 1: A5, 2: F2, 3: F5

    int[] signalFinds = new int[] {0, 0, 0};
    int mostRecentDetection = 0;

    AprilTagDetection tagOfInterest = null;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Timer coneTimer;

    MecanumDrive mecanum;
    Odometry odometry;
    MecanumDrive mecanumDrive;

    Master master;

    public int getStartingPos() {
        return 0;
    }
    @Override
    public void init() {
        startingPos = getStartingPos();

        coneTimer = new Timer();
        telemetry.addLine("Start time: " + coneTimer.getStartTime());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        telemetry.setAutoClear(false);
        mecanum = new MecanumDrive(hardwareMap);
        // odometry = new Odometry(hardwareMap);
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
                telemetry.addLine("Saw tag");
                tagToTelemetry(tagOfInterest);
            }

        }
    }

    double signalLocationX;
    double signalLocationY;

    @Override
    public void start() {
        telemetry.setAutoClear(true);

        telemetry.addData("Signal location", mostRecentDetection);
        telemetry.addData("Signal finds", signalFinds);

        telemetry.update();

        signalLocationX = signalLocations[startingPos][mostRecentDetection - 1].x;
        signalLocationY = signalLocations[startingPos][mostRecentDetection - 1].y;
    }

    @Override
    public void loop() {
        telemetry.addData("Signal #", mostRecentDetection);
        telemetry.addData("Signal finds", "" + signalFinds[0], signalFinds[1], signalFinds[2]);
        telemetry.addData("Signal location", signalLocations[startingPos][mostRecentDetection - 1]);
        telemetry.update();
        double time = coneTimer.getTime();
        if (time > 25) {
            goToSignalLocation((int)odometry.getXCoordinate(), (int) odometry.getYCoordinate(), (int) signalLocationX, (int) signalLocationY);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine("-------------------------");
        telemetry.addLine(String.format("Found tag %d in %.3f seconds", detection.id, coneTimer.getDeltaTime()));
        telemetry.addLine("-------------------------");
    }

    void getCone() {

    }

    void pickUpCone() {

    }

    void goToScoringLocation() {

    }

    void goToConeStack() {

    }

    void scoreCone() {

    }

    void goToSignalLocation(int posX, int posY, int targetX, int targetY) {
        // go to center of square
        int directionX = posX < targetX ? 1 : 0;
        int centerX = (int) (Math.floor(posX / 12d) + directionX) * 12;

        int directionY = posY < targetY ? 1 : 0;
        int centerY = (int) (Math.floor(posY / 12d) + directionY) * 12;

        goToPosition(centerX, posY);
        goToPosition(centerX, centerY);
        goToPosition(centerX, targetY);
        goToPosition(targetX, targetY);
    }

    void goToPosition(int targetX, int targetY) {
        boolean atLocation = false;
        while (!atLocation) {
            atLocation = mecanumDrive.driveTo(targetX, targetY, 0);
        }
    }
}

