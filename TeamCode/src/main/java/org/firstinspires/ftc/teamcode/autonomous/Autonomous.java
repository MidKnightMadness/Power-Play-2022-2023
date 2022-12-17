package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.highlevel.GridSystem;
import org.firstinspires.ftc.teamcode.manipulator.Claw;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.objectdetection.AprilTagDetection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.highlevel.fieldData;
import org.firstinspires.ftc.teamcode.common.Timer;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

import java.util.ArrayList;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode implements cameraInfo, fieldData, pickUpConeData
{
    public int startingPos = 0;  // 0: A2, 1: A5, 2: F2, 3: F5

    public static int numberOfConesInStack = 5;
    int[] signalFinds = new int[] {0, 0, 0};
    int mostRecentDetection = 0;


    Vector2 coneStackLocation = coneStackLocations[startingPos];
    Vector2 scoringLocation = scoringLocations[startingPos];

    AprilTagDetection tagOfInterest = null;

    LinearSlides linearSlides;

    Claw claw;
    GridSystem gridSystem;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Timer coneTimer;
    double time;

    public Odometry odometry;
    public MecanumDrive mecanum;

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

        mecanum = new MecanumDrive(hardwareMap);
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

        // SCORE PRE-LOAD
        // score at terminal
        if (getStartingPos() == 2) {
            goToPosition(getStartingPosition().x + 20, getStartingPosition().y + 1, getStartingRotation());
            goToPosition(getStartingPosition().x, getStartingPosition().y, getStartingRotation());
        } else if (getStartingPos() == 3) {
            goToPosition(getStartingPosition().x - 20, getStartingPosition().y + 1, getStartingRotation());
            goToPosition(getStartingPosition().x, getStartingPosition().y, getStartingRotation());
        }
        sleep(1000);

//        goToPosition(getStartingPosition().x, getStartingPosition().y + 51, getStartingRotation());



//----------------LOOP----------------------------------------------------------------------------------------------------

        while (opModeIsActive()) {
            time = coneTimer.getTime();

//            double [] manipulatorInputs = {0.0, 0.0, 0.0};
//            // score at high junction w/ preloaded cone, will need to take off terminal scoring code
//            claw.closeClaw();
//            goToPosition(getStartingPosition().x, getStartingPosition().y + 51, getStartingRotation());
//            sleep(3000);
//            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians());
//
//            goToPosition(getStartingPosition().x, getStartingPosition().y + 51, manipulatorInputs[0]);
//            sleep(3000);
//            linearSlides.pivotTo(manipulatorInputs[1], telemetry);
//            sleep(3000);
//            linearSlides.extendTo(manipulatorInputs[2], telemetry);
//            sleep(3000);
//            claw.openClaw();
//            sleep(1000);
//
//            sleep(5000);
//            linearSlides.pivotTo(0, telemetry);
//            sleep(3000);
//            linearSlides.extendTo(0, telemetry);
//            sleep(5000);


            goToPosition(getStartingPosition().x, getStartingPosition().y + 30, getStartingRotation());
            if(mostRecentDetection == 1) {
                goToPosition(getStartingPosition().x - 23.5, getStartingPosition().y + 26, getStartingRotation());
                goToPosition(getStartingPosition().x - 23.5, getStartingPosition().y + 32, getStartingRotation());
            } else if (mostRecentDetection == 2) {
                goToPosition(getStartingPosition().x, getStartingPosition().y + 32, getStartingRotation());
            } else if (mostRecentDetection == 3) {
                goToPosition(getStartingPosition().x + 23.5, getStartingPosition().y + 26, getStartingRotation());
                goToPosition(getStartingPosition().x + 23.5, getStartingPosition().y + 32, getStartingRotation());
            }

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

            atLocation = mecanum.driveTo(targetX, targetY, targetAngle, odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians());

            sleep(20);
        }
    }

    // Cycles once to specified junction idicies
    private static double DEFAULT_SCORING_RADIUS = 8.0;
    public void cycle(int targetRow, int targetColumn){ // Indices from
        // Cone pickup
        claw.openClaw();
        claw.closeClaw();

        // Rotate to starting orientation
        goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI / 2);

        sleep(1000);
        //                            __________________________________________
        // ==========================|      NAVIGATING TO SCORING LOCATION      |
        //                            ------------------------------------------

        // Assumes starting at at substation
        // Move to center of square
        goToPosition(odometry.getXCoordinate() - (odometry.getXCoordinate() % 23.50) + 11.75,
                odometry.getYCoordinate() - (odometry.getYCoordinate() % 23.50) + 11.75, odometry.getRotationRadians());

        // Go to x coordinate
        if((targetColumn + 1) * 23.50 >= odometry.getXCoordinate()){ // Case: junction should be right of robot
            goToPosition((targetColumn + 1) * 23.50 - (DEFAULT_SCORING_RADIUS * Math.cos(Math.PI / 4)),
                    odometry.getYCoordinate(), odometry.getRotationRadians());

        }else if((targetColumn + 1) * 23.50 < odometry.getXCoordinate()){ // Case: junction should be left of robot
            goToPosition((targetColumn + 1) * 23.50 + (DEFAULT_SCORING_RADIUS * Math.cos(Math.PI / 4)),
                    odometry.getYCoordinate(), odometry.getRotationRadians());
        }

        // Go to y coordinate
        if((targetRow + 1) * 23.50 >= odometry.getYCoordinate()){ // Case: junction should be in front of robot
            goToPosition(odometry.getXCoordinate(), (targetColumn + 1) * 23.50 - (DEFAULT_SCORING_RADIUS * Math.cos(Math.PI / 4)),
                    odometry.getRotationRadians());

        }else if((targetRow + 1) * 23.50 < odometry.getYCoordinate()){ // Case: junction should be behind robot
            goToPosition(odometry.getXCoordinate(), (targetColumn + 1) * 23.50 + (DEFAULT_SCORING_RADIUS * Math.cos(Math.PI / 4)),
                    odometry.getRotationRadians());
        }

        sleep(1000);
        //                            ____________________________________________
        // ==========================|          TURN AND MOVE MANIPULATOR         |
        //                            --------------------------------------------

        // Turn, pivot manipulator, and extend manipulator
        double [] manipulatorInputs = {0.0, 0.0, 0.0};
        if((targetRow + 1) * 23.50 >= odometry.getYCoordinate() &&
                (targetColumn + 1) * 23.50 >= odometry.getXCoordinate()){ // Top right
            // Aimbot with 45˚
            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI / 4);
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(),
                    manipulatorInputs[0]);
            // Rotate to angle
            linearSlides.pivotTo(manipulatorInputs[1], telemetry);
            linearSlides.extendTo(manipulatorInputs[2], telemetry);


        }else if((targetRow + 1) * 23.50 >= odometry.getYCoordinate() &&
                (targetColumn + 1) * 23.50 < odometry.getXCoordinate()){ // Top left
            // Aimbot with 135˚
            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI * 3 / 4);
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(),
                    manipulatorInputs[0]);
            // Rotate to angle
            linearSlides.pivotTo(manipulatorInputs[1], telemetry);
            linearSlides.extendTo(manipulatorInputs[2], telemetry);

        }else if((targetRow + 1) * 23.50 < odometry.getYCoordinate() &&
                (targetColumn + 1) * 23.50 < odometry.getXCoordinate()){ // Bottom left
            // Aimbot with 225˚
            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI * 5 / 4);
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(),
                    manipulatorInputs[0]);
            // Rotate to angle
            linearSlides.pivotTo(manipulatorInputs[1], telemetry);
            linearSlides.extendTo(manipulatorInputs[2], telemetry);

        }else if((targetRow + 1) * 23.50 < odometry.getYCoordinate() &&
                (targetColumn + 1) * 23.50 >= odometry.getXCoordinate()){ // Bottom right
            // Aimbot with 315˚
            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI * 7 / 4);
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(),
                    manipulatorInputs[0]);
            // Rotate to angle
            linearSlides.pivotTo(manipulatorInputs[1], telemetry);
            linearSlides.extendTo(manipulatorInputs[2], telemetry);
        }

        claw.openClaw();

        sleep(1000);
        //                            ____________________________________________
        // ==========================|            RETURNING TO SUBSTATION         |
        //                            --------------------------------------------

        // Reset for cone pickup
        linearSlides.extendTo(linearSlides.STARTING_EXTENDER_LENGTH, telemetry);
        linearSlides.pivotTo(0, telemetry);
        goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians());

        sleep(1000);

        // Go back to substation location
        // Go to center of square
        goToPosition(odometry.getXCoordinate() - (odometry.getXCoordinate() % 23.50) + 11.75,
                odometry.getYCoordinate() - (odometry.getYCoordinate() % 23.50) + 11.75, odometry.getRotationRadians());

        // x coordinates
        if(23.50 * 3 <= odometry.getXCoordinate()){ // Case: needs to go left
            goToPosition((23.50 * 3) + 11.75, odometry.getYCoordinate(), odometry.getRotationRadians());

        }else if(23.50 * 3 > odometry.getXCoordinate()){ // Case: needs to go right
            goToPosition((23.50 * 3) - 11.75, odometry.getYCoordinate(), odometry.getRotationRadians());
        }

        // y coordinates
        goToPosition(odometry.getXCoordinate(), 23.50, odometry.getRotationRadians());

        // Rotate
        if(23.50 * 3 <= odometry.getXCoordinate()){ // Case: needs to go left
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI * 5 / 4);

        }else if(23.50 * 3 > odometry.getXCoordinate()){ // Case: needs to go right
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI * 7 / 4);
        }
    }

    // Cycle from medium junction across from left
    private int numberOfCycles = 0;
    public static double clawPivotInput = 0.0;
    public void cycleFromLeftMediumJunction() { // Make sure to remove scoring on terminal; CALL BEFORE PARKING
        numberOfCycles++;
        double[] manipulatorInputs = new double[3];
        claw.openClaw();

        // Position robot for picking up cone
        goToPosition(getStartingPosition().x + (23.50 * 2) + 11.75, getStartingPosition().y, getStartingRotation());
        goToPosition(getStartingPosition().x + (23.50 * 2) + 11.75, getStartingPosition().y, Math.PI);
        goToPosition(getStartingPosition().x - 18.75, getStartingPosition().y, Math.PI);
        claw.closeClaw();
        sleep(1000);

        // Move to scoring position for scoring backwards
        manipulatorInputs = GridSystem.pointAtJunction((23.5 * 2) - (DEFAULT_SCORING_RADIUS * Math.cos(Math.PI / 4)), (23.5 * 2) + (DEFAULT_SCORING_RADIUS * Math.sin(Math.PI / 4)), odometry.getRotationRadians());
    }

    public void rotateArmTo(double angle, Telemetry telemetry){
        linearSlides.pivotTo(angle, telemetry);

        // Needs something to get only 0.1, 0.2, 0.3, etc...

//        clawPivotInput = (- slides.seesawAngle / Math.PI + 1) - ((- slides.seesawAngle / Math.PI + 1) % 0.01);

        clawPivotInput = - linearSlides.seesawAngle / Math.PI;
        // -1.0 (undefined position) if at 180˚, 0.0 if at 0˚ (backwards)
        clawPivotInput += 1;
        // 0.0 (backwards) if at 180˚, 1.0 (forwards) if at 0˚

        // Servo only takes inputs in intervals of 0.1
        clawPivotInput = (int) (clawPivotInput * 600.0);
        clawPivotInput /= 1000.0;

        claw.rotateClaw(clawPivotInput);
        // Upper may be 0.8 ish, NOT 1.0
        //  0.0  to  1.0
        // (back) (forward)
    }
}



