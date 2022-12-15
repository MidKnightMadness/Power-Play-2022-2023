package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.highlevel.Master.invSqrt;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class Autonomous extends LinearOpMode implements cameraInfo, fieldData, pickUpConeData
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

    public Odometry odometry;
    public MecanumDrive mecanumDrive;

    private BNO055IMU imu;
    Orientation angles;

    double signalLocationX;
    double signalLocationY;

    public int getStartingPos() {
        return 0;
    }

    public double getStartingRotation() {
        return Math.PI/2;
    }

    public Vector2 getStartingPosition() {
        return new Vector2(35, halfRobotWidth);
    }

    @Override
    public void runOpMode() {


//----------------INIT----------------------------------------------------------------------------------------------------

        mecanumDrive = new MecanumDrive(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        startingPos = getStartingPos();

        coneTimer = new Timer();
        telemetry.addLine("Start time: " + coneTimer.getStartTime());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        mecanumDrive = new MecanumDrive(hardwareMap);
        odometry = new Odometry(hardwareMap, getStartingPosition(), getStartingRotation());

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("ERROR", "Error code" + errorCode);
            }
        });



//----------------INIT-LOOP----------------------------------------------------------------------------------------------------

        while (!isStarted() && !isStopRequested()) {
            coneTimer.getTime();

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                        signalFinds[tag.id - 1] += 1;
                        mostRecentDetection = tag.id;
                        tagOfInterest = tag;
                        tagToTelemetry(tagOfInterest);
                        tagFound = true;
                    }
                }
            }
            odometry.updatePosition();
            telemetry.addLine(String.format("Current Coordinates: (%3.2f, %3.2f)", odometry.getXCoordinate(), odometry.getYCoordinate()));
            telemetry.addLine(String.format("Radians: %3.2f", odometry.getRotationRadians()));
            telemetry.addLine(String.format("Degrees: %3.2f", odometry.getRotationDegrees()));
            telemetry.addData("Signal location", mostRecentDetection);
            telemetry.addData("Signal finds", "" + signalFinds[0], signalFinds[1], signalFinds[2]);

            telemetry.addData("\nPivot Motor reading", linearSlides.seeSawMotor.getCurrentPosition());
            telemetry.addData("Extension Motor 1 reading", linearSlides.extensionMotor.getCurrentPosition());
            telemetry.addData("Extension Motor 2 reading", linearSlides.extensionMotor2.getCurrentPosition());

            telemetry.addData("\nManipulator pivot Angle", linearSlides.seesawAngle * 180 / Math.PI);
            telemetry.addData("\nManipulator extension length", linearSlides.seesawExtensionLength);

            telemetry.update();
            sleep(20);
        }


//----------------START----------------------------------------------------------------------------------------------------

        telemetry.update();

        if (mostRecentDetection != 0) {
            signalLocationX = signalLocations[startingPos][mostRecentDetection - 1].x;
            signalLocationY = signalLocations[startingPos][mostRecentDetection - 1].y;
        } else {
            signalLocationX = signalLocations[startingPos][1].x;
            signalLocationY = signalLocations[startingPos][1].y;
        }

//        goToScoringLocation();
        goToPosition(getStartingPosition().x, getStartingPosition().y + 51, getStartingRotation());
        sleep(3000);
        goToPosition(getStartingPosition().x, getStartingPosition().y + 51, getStartingRotation() - Math.PI / 4);
        sleep(3000);
        goToPosition(getStartingPosition().x, getStartingPosition().y + 51, getStartingRotation());
        sleep(3000);
//        linearSlides.scoreFromDefaultScoringPosition();


//----------------LOOP----------------------------------------------------------------------------------------------------

        while (opModeIsActive()) {
            time = coneTimer.getTime();

//            if (time > 26.35729278100687712039158d) {
//                goToPosition(getStartingPosition().x, getStartingPosition().y + 26, getStartingRotation());
//                sleep(3000);
//                if(mostRecentDetection == 1) {
//                    goToPosition(getStartingPosition().x - 23.5, getStartingPosition().y + 26, getStartingRotation());
//                } else if (mostRecentDetection == 3) {
//                    goToPosition(getStartingPosition().x + 23.5, getStartingPosition().y + 26, getStartingRotation());
//                }
//            }

                // Making sure cone is grabbed, needs to lift manipulator off support first
                claw.openClaw();
                linearSlides.pivotTo(Math.PI / 2);
                sleep(3000);
                linearSlides.pivotTo(0);
                claw.closeClaw();
                sleep(3000);

                // Scoring preloaded cone
                goToPosition(getStartingPosition().x, getStartingPosition().y + 60.375, getStartingRotation());
                sleep(5000);
                goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(), 1.107148718); // Junction
                sleep(5000);
                linearSlides.pivotTo(1.174268847);
                sleep(5000);
                linearSlides.extendTo(34.01424334);
                sleep(5000);
                claw.openClaw();



//                goToSignalLocation((int)odometry.getXCoordinate(), (int) odometry.getYCoordinate(), (int) signalLocationX, (int) signalLocationY);
                requestOpModeStop();
//            } else {
//                try {
//                    cycle();
//                }
//                catch (InterruptedException e) {
//                    telemetry.addLine(e.toString());
//                    telemetry.update();
//                }
//            }
            sleep(20);
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

        sleep(1000);

        claw.openClaw();
        sleep(100);
        claw.closeClaw();
    }

    void cycle() throws InterruptedException{
        linearSlides.grabFromDefaultScoringPosition();
        sleep(100);
        linearSlides.scoreFromDefaultScoringPosition();
        sleep(100);
//
//        if(Vector.lengthOf(Vector.add(Vector.neg(linearSlides.getClawCoordinates()), linearSlides.DEFAULT_SCORING_DISPLACEMENT)) > 0.1){
//
//        }
    }

    void goToScoringLocation() {
        goToPosition((int) scoringLocation.x, (int) scoringLocation.y, 0);
    }

    void goToConeStack() {
        goToPosition((int) coneStackLocation.x, (int) scoringLocation.y, 0);
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
            if (mostRecentDetection != 0) { telemetry.addLine("SIGNAL TAG FOUND, GOING TO POSITION " + mostRecentDetection); }
            else { telemetry.addLine("SIGNAL TAG NOT FOUND, GOING TO POSITION 2"); }
            telemetry.addData("\nCone timer", coneTimer.getTime());

            telemetry.addLine(String.format("\nCurrent Coordinates: (%3.2f, %3.2f, %3.2f)", odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians()));
            telemetry.addLine(String.format("Target Coordinates: (%3.2f, %3.2f, %3.2f)", targetX, targetY, targetAngle));
            telemetry.addLine(String.format("Target - current: (%3.2f, %3.2f, %3.2f)", targetX - odometry.getXCoordinate(), targetY - odometry.getYCoordinate(), targetAngle - odometry.getRotationRadians()));
            telemetry.addData("Signal #", mostRecentDetection);
            telemetry.addData("Signal finds", "" + signalFinds[0], signalFinds[1], signalFinds[2]);
            if (mostRecentDetection != 0)
                telemetry.addData("Signal location", signalLocations[startingPos][mostRecentDetection - 1]);
            telemetry.update();

            odometry.updatePosition();

            atLocation = mecanumDrive.driveTo(targetX, targetY, targetAngle, odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians());

        }
    }

}

