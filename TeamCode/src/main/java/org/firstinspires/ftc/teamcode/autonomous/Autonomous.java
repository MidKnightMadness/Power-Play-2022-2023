package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.highlevel.Master.invSqrt;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

import java.util.ArrayList;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends OpMode implements cameraInfo, fieldData, pickUpConeData
{
    public int startingPos = 0;  // 0: A2, 1: A5, 2: F2, 3: F5
    Thread thread;

    public static int numberOfConesInStack = 5;
    int[] signalFinds = new int[] {0, 0, 0};
    int mostRecentDetection = 0;


    Vector2 coneStackLocation = coneStackLocations[startingPos];
    Vector2 scoringLocation = scoringLocations[startingPos];

    AprilTagDetection tagOfInterest = null;

    LinearSlides linearSlides;

    Claw claw;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Timer coneTimer;
    double time;

    Odometry odometry;
    static MecanumDrive mecanumDrive;

    private BNO055IMU imu;
    Orientation angles;

    public int getStartingPos() {
        return 0;
    }

    public double getStartingRotation() {
        return Math.PI / 2;
    }

    public Vector2 getStartingPostition() {
        return new Vector2(7.5, realSquareWidth * 1.5);
    }

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap);

        startingPos = getStartingPos();

        coneTimer = new Timer();
        telemetry.addLine("Start time: " + coneTimer.getStartTime());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//        telemetry.setAutoClear(false);

        mecanumDrive = new MecanumDrive(hardwareMap);

        odometry = new Odometry(hardwareMap, getStartingRotation(), getStartingPostition());
        odometry.resetEncoders();
        odometry.setPostion(getStartingPostition());
        odometry.setRotation(getStartingRotation());

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

        // Resetting positions for odometry

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
                    tagToTelemetry(tagOfInterest);
                    tagFound = true;
                }
            }
        }
        telemetry.addData("Signal location", mostRecentDetection);
        telemetry.addData("Signal finds", signalFinds);
//        odometry.updatePosition();
        odometry.telemetry(telemetry);
        telemetry.addLine("Scoring location " + scoringLocation.toString());
        telemetry.addData("expected", getStartingRotation() + " " + getStartingPostition().toString());
        telemetry.update();
    }

    double signalLocationX;
    double signalLocationY;

    // Linked list of poses (3 element arrays)
    public static ArrayList <double []> AutonomousPoseArray = new ArrayList<>();

    @Override
    public void start() {
//        Autonomous.AutonomousPoseArray.add(new double [] {0, 0, 0}); // Default
//        Autonomous.AutonomousPoseArray.add(new double [] {0, 12, 0}); // 1 foot up
        odometry.position = getStartingPostition();
        odometry.rotationRadians = getStartingRotation();

        telemetry.setAutoClear(false);

        telemetry.addData("Signal location", mostRecentDetection);
        telemetry.addData("Signal finds", signalFinds);

        telemetry.update();

        if (mostRecentDetection != 0) {
            signalLocationX = signalLocations[startingPos][mostRecentDetection - 1].x;
            signalLocationY = signalLocations[startingPos][mostRecentDetection - 1].y;
            telemetry.addLine("SIGNAL TAG FOUND, GOING TO POSITION " + mostRecentDetection);
        }
        else {
            signalLocationX = signalLocations[startingPos][1].x;
            signalLocationY = signalLocations[startingPos][1].y;

            telemetry.addLine("SIGNAL TAG NOT FOUND, GOING TO POSITION 2");
        }

        telemetry.setAutoClear(true);

        // Testing autonomous
         /*AutonomousPoseArray.get(1)[0],
                AutonomousPoseArray.get(1)[1],
                AutonomousPoseArray.get(1)[2]);*/

//        goToScoringLocation();
//        linearSlides.scoreFromDefaultScoringPosition();
    }

    @Override
    public void loop() { // Analogous to while(active){
        goToScoringLocation();
        try {
            Thread.sleep(1000);
        }
        catch (InterruptedException e) {
            telemetry.addLine("ERROR " + e.toString());
        }

        goToPosition(getStartingPostition().x, getStartingPostition().y, 0);

        requestOpModeStop();

//        telemetry.addData("Signal #", mostRecentDetection);
//        telemetry.addData("Signal finds", "" + signalFinds[0], signalFinds[1], signalFinds[2]);
//        telemetry.addData("Signal location", signalLocations[startingPos][mostRecentDetection - 1]);

        time = coneTimer.getTime();

//        if (time > 26.35729278100687712039158d) {
//            goToSignalLocation((int)odometry.getXCoordinate(), (int) odometry.getYCoordinate(), (int) signalLocationX, (int) signalLocationY);
//            requestOpModeStop();
//        } else {
//            try {
//                cycle();
//            }
//            catch (InterruptedException e) {
//                telemetry.addLine(e.toString());
//                telemetry.update();
//            }
//        }

        telemetry.addLine("\n\n");
        Autonomous.mecanumDrive.telemetry(this.telemetry);
        telemetry.update();
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

    void cycle() throws InterruptedException{
        linearSlides.grabFromDefaultScoringPosition();
        thread.sleep(100);
        linearSlides.scoreFromDefaultScoringPosition();
        thread.sleep(100);
//
//        if(Vector.lengthOf(Vector.add(Vector.neg(linearSlides.getClawCoordinates()), linearSlides.DEFAULT_SCORING_DISPLACEMENT)) > 0.1){
//
//        }
    }

    void goToScoringLocation() {
        goToPosition((int) scoringLocation.x, (int) scoringLocation.y, Math.PI / 2);
    }

    void goToConeStack() {
        goToPosition((int) coneStackLocation.x, (int) scoringLocation.y, Math.PI / 2);
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

        goToPosition(centerX, posY, 0);
        goToPosition(centerX, centerY, 0);
        goToPosition(centerX, targetY, 0);
        goToPosition(targetX, targetY, 0);
    }

    void goToPosition(double targetX, double targetY, double targetAngle) {
        boolean atLocation = false;

        while (!atLocation) {
            telemetry.addData("At Location", atLocation);
            telemetry.addLine(String.format("Current Coordinates: (%3.2f, %3.2f, %3.2f)", odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationDegrees()));
            telemetry.addLine(String.format("Target Coordinates: (%3.2f, %3.2f, %3.2f)", targetX, targetY, targetAngle));
            telemetry.addLine(String.format("Target - current: (%3.2f, %3.2f, %3.2f)", targetX - odometry.getXCoordinate(), targetY - odometry.getYCoordinate(), targetAngle - odometry.getRotationDegrees()));

            telemetry.update();

            odometry.updatePosition();

            atLocation = mecanumDrive.driveTo(targetX, targetY, targetAngle, odometry.getXCoordinate(), odometry.getYCoordinate(), -odometry.getRotationRadians());

        }

        telemetry.addData("At Location", atLocation);
        telemetry.update();

    }

}

