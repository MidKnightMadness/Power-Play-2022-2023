package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.manipulator.Claw;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drivetrain.Vector;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.highlevel.Master;
import org.firstinspires.ftc.teamcode.highlevel.fieldData;
import org.firstinspires.ftc.teamcode.common.Timer;
import org.firstinspires.ftc.teamcode.odometry.Vector2;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends OpMode implements cameraInfo, fieldData, pickUpConeData
{
    public int startingPos = 0;  // 0: A2, 1: A5, 2: F2, 3: F5
    Thread thread;

    public static int numberOfConesInStack = 5;
    int[] signalFinds = new int[] {0, 0, 0};
    int mostRecentDetection = 0;

    double [] coneStackLocation = coneStackLocations[startingPos];
    double [] scoringLocation = scoringLocations[startingPos];

    AprilTagDetection tagOfInterest = null;

    LinearSlides linearSlides;

    Claw claw;

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
        
        scoreCone();
    }

    @Override
    public void loop() {
        telemetry.addData("Signal #", mostRecentDetection);
        telemetry.addData("Signal finds", "" + signalFinds[0], signalFinds[1], signalFinds[2]);
        telemetry.addData("Signal location", signalLocations[startingPos][mostRecentDetection - 1]);
        telemetry.update();
        double time = coneTimer.getTime();

        if (time < 25) {
            goToSignalLocation((int)odometry.getXCoordinate(), (int) odometry.getYCoordinate(), (int) signalLocationX, (int) signalLocationY);
            requestOpModeStop();
        }
        else {
            try {
                pickUpCone();
            } catch (InterruptedException e) {
                telemetry.addLine(e.toString());
                telemetry.update();
            }
            scoreCone();

        }

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine("-------------------------");
        telemetry.addLine(String.format("Found tag %d in %.3f seconds", detection.id, coneTimer.getDeltaTime()));
        telemetry.addLine("-------------------------");
    }


    int upDown = startingPos == 0 || startingPos == 2 ? -1 : 1;

    void pickUpCone() throws InterruptedException {
        double coneHeight = INITIAL_CONE_ABOVE_GROUND + INCHES_ABOVE_CONE + INCHES_ABOVE_PER_CONE * numberOfConesInStack;

        goToConeStack();
        linearSlides.goPointAt(new double[] {0, upDown * 12 , coneHeight });

        thread.sleep(1000);

        claw.openClaw();
        thread.sleep(100);
        claw.closeClaw();
    }

    void cycle() {

    }

    void goToScoringLocation() {
        goToPosition((int) scoringLocation[0], (int) scoringLocation[1]);
    }

    void goToConeStack() {
        goToPosition((int) coneStackLocation[0], (int) scoringLocation[1]);
    }


    void scoreCone() {
        double junctionHeight = junctionHeights[1][2];
        goToScoringLocation();

        linearSlides.goPointAt(new double[] {upDown * 12, 0, junctionHeight});
        claw.openClaw();
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

