package org.firstinspires.ftc.teamcode.currentOpModes;

import static org.firstinspires.ftc.teamcode.highlevel.GridSystem.pointAtJunction;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.Timer;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.manipulator.Claw;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

/*
 * Control Hub Configurations
 * Motors:
 * 0    FL      REV Robotics 20:1 HD Hex Motor
 * 1    BR      REV Robotics 20:1 HD Hex Motor
 * 2    BL      REV Robotics 20:1 HD Hex Motor
 * 3    FR      REV Robotics 20:1 HD Hex Motor
 *
 * Expansion Hub Configurations
 * Motors:
 * 0    SSM     REV Robotics 20:1 HD Hex Motor
 * 1    LSEM    NeveRest 20 Gearmotor
 * 2    LSEM2   NeveRest 20 Gearmotor
 * Servos:
 * 0    Claw    Servo
 * 1    CP      CRServo
 * 2    CP2     CRServo
 */

/*
 * Controls
 * Player 1:
 * Player 2:
 *
 */


@TeleOp(name="Main")
public class MainTeleOp extends OpMode {
    public static MecanumDrive mecanum;
    public static Odometry odometry;
//    LinearSlides slides;
//    Claw claw;

    public static double [] currentPosition = {0.0, 0.0};

    // First controller left stick
    double[] lastInputs = {0, 0};
    double[] currentInputs = {0, 0};

    Timer timer;

    final double DEADZONE_TOLERANCE = 0.05;

    private boolean lastPressedDriveMode = false;
    private boolean driveModeToggle = false;
    private boolean lastClawOpenToggle = false;
    private boolean isClawOpenToggle = false;

    //testing ??
    private boolean lastPressedSeesaw = false;
    private boolean seesawToggle = false;
    private boolean lastPressedLinearSlides = false;
    private boolean linearSlidesToggle = false;



    @Override
    public void init() {
        timer = new Timer();

        mecanum = new MecanumDrive(hardwareMap);
        odometry = new Odometry(hardwareMap, new Vector2(0, 0), Math.PI / 2);
//        claw = new Claw(hardwareMap);
//        slides = new LinearSlides(hardwareMap);
    }

    double time;
    double deltaTime;
    double previousInputWeight = 0.95;
    double powerMultiplier = 1.0;

    @Override
    public void loop() {


        // DRIVER ASSIST
        if (gamepad1.x || gamepad1.square) {
            odometry.resetEncoders();
        }

//        if (gamepad2.x) {
//            slides.resetEncoders();
//        }

        if (gamepad1.a || gamepad1.cross) {
            previousInputWeight += 0.01;
            if (previousInputWeight > 1) {
                previousInputWeight = 1;
            }
            try {
                Thread.sleep(75);
            } catch (InterruptedException e) {
                telemetry.addLine(e.toString());
            }
        }

        if (gamepad1.b || gamepad1.circle) {
            previousInputWeight -= 0.01;
            if (previousInputWeight < 0) {
                previousInputWeight = 0;
            }
            try {
                Thread.sleep(75);
            } catch (InterruptedException e) {
            }
        }

        currentInputs[0] = gamepad1.left_stick_x * previousInputWeight + lastInputs[0] * (1 - previousInputWeight);
        currentInputs[1] = gamepad1.left_stick_y * previousInputWeight + lastInputs[1] * (1 - previousInputWeight);

        lastInputs[0] = currentInputs[0];
        lastInputs[1] = currentInputs[1];




        // DRIVE
        if (gamepad1.left_bumper && !lastPressedDriveMode) {
            driveModeToggle = !driveModeToggle;
        }
        lastPressedDriveMode = gamepad1.left_bumper;

        if (driveModeToggle) {
            mecanum.fieldOrientatedDrive(currentInputs[0] * powerMultiplier, -currentInputs[1] * powerMultiplier,
                    (gamepad1.right_stick_x + gamepad2.left_stick_x) * powerMultiplier, odometry.getRotationRadians());

//            if (gamepad1.dpad_up) { mecanum.fieldOrientatedDrive(0, -powerMultiplier, 0); }
//            if (gamepad1.dpad_down) { mecanum.fieldOrientatedDrive(0, powerMultiplier, 0); }
//            if (gamepad1.dpad_right) { mecanum.fieldOrientatedDrive(powerMultiplier, 0, 0); }
//            if (gamepad1.dpad_left) { mecanum.fieldOrientatedDrive(-powerMultiplier, 0, 0); }
        } else {
            mecanum.drive(currentInputs[0] * powerMultiplier, -currentInputs[1] * powerMultiplier,
                    (gamepad1.right_stick_x + gamepad2.left_stick_x) * powerMultiplier); // normal drive

//            if (gamepad1.dpad_up) { mecanum.drive(0, -powerMultiplier, 0); }
//            if (gamepad1.dpad_down) { mecanum.drive(0, powerMultiplier, 0); }
//            if (gamepad1.dpad_right) { mecanum.drive(powerMultiplier, 0, 0); }
//            if (gamepad1.dpad_left) { mecanum.drive(-powerMultiplier, 0, 0); }
        }




        // LINEAR SLIDES
//        slides.extendBy(-gamepad2.right_stick_y);
//
//        if (gamepad2.y && !lastPressedLinearSlides) {
//            linearSlidesToggle = !linearSlidesToggle;
//        }
//        lastPressedLinearSlides = gamepad2.y;
//
//        if (linearSlidesToggle) {
//            slides.extendTo(6);
//        } else {
//            slides.extendTo(0);
//        }




        // SEESAW
//        slides.pivotBy(gamepad2.left_stick_y);
//
//        if (gamepad2.b && !lastPressedSeesaw) {
//            seesawToggle = !seesawToggle;
//        }
//        lastPressedSeesaw = gamepad2.b;
//
//        if (seesawToggle) {
//            slides.pivotTo(Math.PI);
//        } else {
//            slides.pivotTo(0);
//        }





//        // CLAW
//        if (gamepad2.right_bumper && !lastClawOpenToggle) {
//            isClawOpenToggle = !isClawOpenToggle;
//        }
//        lastClawOpenToggle = gamepad2.right_bumper;
//
//        if (isClawOpenToggle) {
//            claw.openClaw();
//        } else {
//            claw.closeClaw();
//        }
//
//
//
//        // claw pivot
//        if (gamepad2.dpad_up) {
//            claw.rotateClaw(1);
//        } else if (gamepad2.dpad_down) {
//            claw.rotateClaw(-1);
//        } else {
//            claw.rotateClaw(0);
//        }

        // Adjusting angle target
        if(gamepad1.left_trigger >= 0.5){ // Adjsting angle
            if(gamepad1.square){
                targetAngle -= 0.001;
            }else if(gamepad1.circle){
                targetAngle += 0.001;
            }
        }

        if (gamepad1.triangle) {
            while(Math.abs(pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians())[0]) - Math.abs(odometry.getRotationRadians()) > 0.08){ // Not at aimbot angle
                if(pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians())[0] > odometry.getRotationRadians()){
                    mecanum.drive(0.0, 0.0, 0.3);
                }else{
                    mecanum.drive(0.0, 0.0, 0.3);
                }

                if(gamepad1.left_trigger > 1.0){
                    break;
                }
            }
        }

        // Autonomous testing
        if(gamepad1.dpad_right){
            goToPosition(odometry.getXCoordinate() + 12, odometry.getYCoordinate() , targetAngle);
        }else if(gamepad1.dpad_left){
            goToPosition(odometry.getXCoordinate() - 12, odometry.getYCoordinate(), targetAngle);
        }else if(gamepad1.dpad_up){
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate() + 12, targetAngle);
        }else if(gamepad1.dpad_down){
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate() - 12, targetAngle);
        }

        telemetry.addData("\nController target angle", targetAngle);

        odometry.updatePosition();
        telemetry();
    }

    static double targetAngle = 0.0;
    static boolean cancel = false;
    public static boolean newFieldOriented = false;

    // TELEMETRY
    public void telemetry() {
        telemetry.addData("DRIVE MODE", driveModeToggle ? "FIELD ORIENTED": "NORMAL");
        telemetry.addData("New field oriented version", newFieldOriented);

        telemetry.addLine(String.format("Position: [%5.2f, %5.2f]", odometry.getXCoordinate(), odometry.getYCoordinate())); // Check if x and y are still reversed
        telemetry.addData("Angle", odometry.getRotationRadians() * 180 / Math.PI);
        telemetry.addLine("EASE COEFFICIENT " + previousInputWeight);

//        odometry.telemetry(telemetry);
        mecanum.telemetry(telemetry);
//        slides.telemetry(telemetry);
//        claw.telemetry(telemetry);

        telemetry.addLine("\nTIMER");
        telemetry.addLine("DeltaTime " + deltaTime);
        telemetry.addLine("Time" + time);
        telemetry.update();
    }

    double deadZone(double input) {
        if (Math.abs(input) < DEADZONE_TOLERANCE) {
            return 0;
        }

        return input;
    }

    void goToPosition(double targetX, double targetY, double targetAngle) {
        boolean atLocation = false;

        while (!atLocation) {
            if(gamepad1.left_trigger >= 0.5){
                atLocation = true;
               break;
            }
//            telemetry.addData("Precise position adjustment", preciseDisplacement);
//            telemetry.addData("Precise rotation adjustment", preciseRotation);
            telemetry.addData("At Location", atLocation);
            telemetry.addLine(String.format("Current Coordinates: (%3.2f, %3.2f, %3.2f)", odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationDegrees()));
            telemetry.addLine(String.format("Target Coordinates: (%3.2f, %3.2f, %3.2f)", targetX, targetY, targetAngle));
            telemetry.addLine(String.format("Target - current: (%3.2f, %3.2f, %3.2f)", targetX - odometry.getXCoordinate(), targetY - odometry.getYCoordinate(), targetAngle - odometry.getRotationDegrees()));
//            odometry.telemetry(telemetry);
            telemetry.update();

            odometry.updatePosition();

            atLocation = mecanum.driveTo(targetX, targetY, targetAngle, odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians());

        }
        telemetry.addData("At Location", atLocation);
        telemetry.update();

    }

}
