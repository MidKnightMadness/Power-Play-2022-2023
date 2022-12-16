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
    LinearSlides slides;
    Claw claw;

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
        claw = new Claw(hardwareMap);
        slides = new LinearSlides(hardwareMap);
    }

    double time;
    double deltaTime;
    double previousInputWeight = 0.95;
    final double staticPowerMultiplier = 1.0;
    double powerMultiplier = staticPowerMultiplier;

    @Override
    public void loop() {


        // DRIVER ASSIST
        if (gamepad1.x || gamepad1.square) {
            odometry.resetEncoders();
        }

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

        // slow mode (change driving power multiplier)
        powerMultiplier = staticPowerMultiplier * (1 - gamepad1.right_trigger * 0.95); // slows the driving as trigger is pressed



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
        slides.extendBy(-gamepad2.right_stick_y);
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
        slides.pivotBy(-gamepad2.left_stick_y, claw);
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





        // CLAW
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



        // claw pivot
        if (gamepad2.dpad_up) {
            claw.rotateClaw(1);
        } else if (gamepad2.dpad_down) {
            claw.rotateClaw(-1);
        }

        // Adjusting angle target
//        if(gamepad1.left_trigger >= 0.5){ // Adjsting angle
//            if(gamepad1.square){
//                targetAngle -= 0.001;
//            }else if(gamepad1.circle){
//                targetAngle += 0.001;
//            }
//        }

//        if (gamepad1.triangle) {
//            while(Math.abs(pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians())[0]) - Math.abs(odometry.getRotationRadians()) > 0.08){ // Not at aimbot angle
//                if(pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians())[0] > odometry.getRotationRadians()){
//                    mecanum.drive(0.0, 0.0, 0.3);
//                }else{
//                    mecanum.drive(0.0, 0.0, 0.3);
//                }
//
//                if(gamepad1.left_trigger > 1.0){
//                    break;
//                }
//            }
//        }

        // Adjusting extension length and angle
//        if(gamepad2.left_trigger > 0.5){
//            adjustingExtensionLength = !adjustingExtensionLength;
//        }
//        if(gamepad2.dpad_up && adjustingExtensionLength){
//            targetExtension += 0.001;
//        }else if(gamepad2.dpad_down && adjustingExtensionLength){
//            targetExtension -= 0.001;
//        }else if(gamepad2.dpad_up && !adjustingExtensionLength){
//            targetAngle += 0.001;
//        }else if(gamepad2.dpad_down && !adjustingExtensionLength){
//            targetAngle -= 0.001;
//        }

        if(gamepad2.right_trigger > 0.5){
            while(true){
                slides.pivotTo(targetAngle);
                slides.extendTo(targetAngle);

                telemetry.addData("\nController target angle (degrees)", targetAngle * 180 / Math.PI);
                telemetry.addData("Controller target extension length", targetExtension);
                telemetry.addData("Adjusting extension length", adjustingExtensionLength);

                slides.update();
                telemetry.addData("\nPivot angle (degrees)", slides.seesawAngle * 180 / Math.PI);
                telemetry.addData("Extended length", slides.seesawExtensionLength);
                telemetry.update();

                if(gamepad1.left_trigger > 0.5){
                    break;
                }
            }
        }

        telemetry.addData("Pivot Motor reading", slides.seeSawMotor.getCurrentPosition());
        telemetry.addData("Extension Motor 1 reading", slides.extensionMotor.getCurrentPosition());
        telemetry.addData("Extension Motor 2 reading", slides.extensionMotor2.getCurrentPosition());
        telemetry.addData("power", -gamepad2.right_stick_y);
        telemetry.addData("Left stick y", -gamepad2.left_stick_y);

        telemetry.addData("\nController target angle (degrees)", targetAngle * 180 / Math.PI);
        telemetry.addData("Controller target extension length", targetExtension);
        telemetry.addData("Adjusting extension length", adjustingExtensionLength);

        telemetry.addData("\nClaw pivot ticks", claw.rotationServo.getPosition());



        slides.update();
        telemetry.addData("\nPivot angle (degrees)", slides.seesawAngle * 180 / Math.PI);
        telemetry.addData("Extended length", slides.seesawExtensionLength);

//        odometry.updatePosition();
        telemetry.update();
    }

    public static boolean adjustingExtensionLength = false;
    public static double targetExtension = 19.0;
    public static double targetAngle = 0.0;

    // TELEMETRY
    public void telemetry() {
        telemetry.addData("DRIVE MODE", driveModeToggle ? "FIELD ORIENTED": "NORMAL");

        telemetry.addLine(String.format("\nPosition: [%5.2f, %5.2f]", odometry.getXCoordinate(), odometry.getYCoordinate())); // Check if x and y are still reversed
        telemetry.addData("Angle", odometry.getRotationRadians() * 180 / Math.PI);
        telemetry.addLine("Ease Coefficient " + previousInputWeight);
        telemetry.addLine("Power Multiplier " + powerMultiplier +  (1 - gamepad1.right_trigger * 0.95));

//        odometry.telemetry(telemetry);
        mecanum.telemetry(telemetry);
        slides.telemetry(telemetry);
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

    public void rotateArm(double power){
        slides.pivotBy(power, claw);
        claw.rotateClaw(-power* LinearSlides.SEESAW_OVERALL_RATIO / Math.PI);
    }

}
