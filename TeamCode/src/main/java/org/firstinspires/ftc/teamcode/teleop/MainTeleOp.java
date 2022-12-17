package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Timer;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.highlevel.GridSystem;
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
 * 
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

    final double DEADZONE_TOLERANCE = 0.05;

    private boolean lastPressedDriveMode = false;
    private boolean driveModeToggle = false;
    private boolean lastPressedClawOpen = false;
    private boolean clawOpenToggle = false;
    private boolean lastPressedClawPivot = false;
    private boolean clawPivotToggle = false;

    //testing ??
    private boolean lastPressedSeesaw = false;
    private boolean seesawToggle = false;
    private boolean lastPressedLinearSlides = false;
    private boolean linearSlidesToggle = false;



    @Override
    public void init() {
        Timer timer = new Timer();

        mecanum = new MecanumDrive(hardwareMap);
        odometry = new Odometry(hardwareMap, new Vector2(0, 0), Math.PI / 2);
        claw = new Claw(hardwareMap);
        slides = new LinearSlides(hardwareMap);

        // Taking manipulator off of support
//        rotateArmTo(Math.PI / 4, telemetry);
//        rotateArmTo(0, telemetry);
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
                sleep(75);
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
                sleep(75);
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



        // SEESAW
        rotateArm(gamepad2.left_stick_y);



        // CLAW
        if (gamepad2.right_bumper && !lastPressedClawOpen) {
            clawOpenToggle = !clawOpenToggle;
        }
        lastPressedClawOpen = gamepad2.right_bumper;

        if (clawOpenToggle) {
            claw.openClaw();
        } else {
            claw.closeClaw();
        }



        // claw pivot
        if (gamepad2.left_bumper && !lastPressedClawPivot) {
            clawPivotToggle = !clawPivotToggle;
        }
        lastPressedClawPivot = gamepad2.left_bumper;

        if (clawPivotToggle) {
            claw.rotateClaw(0);
        } //else if (!clawPivotToggle){
//            claw.rotateClaw(1);
//        }


        // Adjusting extension length and angle
        if(gamepad2.left_trigger > 0.5){
            adjustingExtensionLength = !adjustingExtensionLength;
        }
        if(gamepad2.dpad_up && adjustingExtensionLength){
            targetExtension += 0.1;
        }else if(gamepad2.dpad_down && adjustingExtensionLength){
            targetExtension -= 0.1;
        }else if(gamepad2.dpad_up && !adjustingExtensionLength){
            targetAngle += 0.1;
        }else if(gamepad2.dpad_down && !adjustingExtensionLength){
            targetAngle -= 0.1;
        }


            while(gamepad2.right_trigger > 0.5){
                rotateArmTo(targetAngle, telemetry);
                slides.extendTo(targetExtension, telemetry);

                telemetry.addData("\nController target angle (degrees)", targetAngle * 180 / Math.PI);
                telemetry.addData("Controller target extension length", targetExtension);
                telemetry.addData("Adjusting extension length", adjustingExtensionLength);

                telemetry.addData("\nPivot Motor reading", slides.seeSawMotor.getCurrentPosition());
                telemetry.addData("Extension Motor 1 reading", slides.extensionMotor.getCurrentPosition());
                telemetry.addData("Extension Motor 2 reading", slides.extensionMotor2.getCurrentPosition());

                slides.update();
                telemetry.addData("\nPivot angle (degrees)", slides.seesawAngle * 180 / Math.PI);
                telemetry.addData("Extended length", slides.seesawExtensionLength);
                telemetry.update();
            }


        telemetry.addData("Pivot Motor reading", slides.seeSawMotor.getCurrentPosition());
        telemetry.addData("Extension Motor 1 reading", slides.extensionMotor.getCurrentPosition());
        telemetry.addData("Extension Motor 2 reading", slides.extensionMotor2.getCurrentPosition());
        telemetry.addData("power", -gamepad2.right_stick_y);
        telemetry.addData("Left stick y", -gamepad2.left_stick_y);

        telemetry.addData("\nController target angle (degrees)", targetAngle * 180 / Math.PI);
        telemetry.addData("Controller target extension length", targetExtension);
        telemetry.addData("Adjusting extension length", adjustingExtensionLength);

        telemetry.addData("\nClaw pivot ticks", clawPivotInput);
        telemetry.addData("Manual claw control ticks", gamepad2.left_trigger);

        slides.update();
        telemetry.addData("\nPivot angle (degrees)", slides.seesawAngle * 180 / Math.PI);
        telemetry.addData("Extended length", slides.seesawExtensionLength);

        odometry.updatePosition();
        telemetry();
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

    public static double clawPivotInput = 0.0;

    public void rotateArm(double power){
        slides.pivotBy(power);

        // Needs something to get only 0.1, 0.2, 0.3, etc...

//        clawPivotInput = (- slides.seesawAngle / Math.PI + 1) - ((- slides.seesawAngle / Math.PI + 1) % 0.01);

        clawPivotInput = - slides.seesawAngle / Math.PI;
        // -1.0 (undefined position) if at 180˚, 0.0 if at 0˚ (backwards)
        clawPivotInput += 1;
        // 0.0 (backwards) if at 180˚, 1.0 (forwards) if at 0˚

        // Servo only takes inputs in intervals of 0.1
        clawPivotInput = (int) (clawPivotInput * 6.0);
        clawPivotInput /= 10.0;

        claw.rotateClaw(clawPivotInput);
        // Upper may be 0.8 ish, NOT 1.0
        //  0.0  to  1.0
        // (back) (forward)
    }

    public void rotateArmTo(double angle, Telemetry telemetry){
        slides.pivotTo(angle, telemetry);

        // Needs something to get only 0.1, 0.2, 0.3, etc...

//        clawPivotInput = (- slides.seesawAngle / Math.PI + 1) - ((- slides.seesawAngle / Math.PI + 1) % 0.01);

        clawPivotInput = - slides.seesawAngle / Math.PI;
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

    // Cycles once to closest tall junction to substation
    public void cycleToClosestTallJunction(){
        // Grab cone
        claw.openClaw();
        slides.pivotTo(0, telemetry);
        slides.extendTo(34 - (7.5 - 3.5), telemetry);
        claw.closeClaw();

        // Raise to position
        slides.pivotTo(Math.PI - Math.atan((37.5 - slides.ROOT_HEIGHT) / (7.5 - 3.5)), telemetry);
        slides.extendTo(Math.sqrt((37.5 - slides.ROOT_HEIGHT) * (37.5 - slides.ROOT_HEIGHT)  + (7.5 - 3.5) * (7.5 - 3.5)), telemetry);
        while((!slides.seeSawMotor.isBusy() && !slides.extensionMotor.isBusy() && !slides.extensionMotor2.isBusy())){
            claw.openClaw();
        }
    }

    // Cycles once to specified junction idicies
    private static double DEFAULT_SCORING_RADIUS = 21.0;
    public void cycle(int targetRow, int targetColumn){ // Row number, column number
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

        // Turn, pivot manipulator, and extend manipulator
        double [] manipulatorInputs = {0.0, 0.0, 0.0};
        if((targetRow + 1) * 23.50 >= odometry.getYCoordinate() &&
                (targetColumn + 1) * 23.50 >= odometry.getXCoordinate()){ // Top right
            // Aimbot with 45˚
            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI / 4);
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(),
                    manipulatorInputs[0]);
            // Rotate to angle
            slides.pivotTo(manipulatorInputs[1], telemetry);
            slides.extendTo(manipulatorInputs[2], telemetry);


        }else if((targetRow + 1) * 23.50 >= odometry.getYCoordinate() &&
                (targetColumn + 1) * 23.50 < odometry.getXCoordinate()){ // Top left
            // Aimbot with 135˚
            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI * 3 / 4);
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(),
                    manipulatorInputs[0]);
            // Rotate to angle
            slides.pivotTo(manipulatorInputs[1], telemetry);
            slides.extendTo(manipulatorInputs[2], telemetry);

        }else if((targetRow + 1) * 23.50 < odometry.getYCoordinate() &&
                (targetColumn + 1) * 23.50 < odometry.getXCoordinate()){ // Bottom left
            // Aimbot with 225˚
            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI * 5 / 4);
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(),
                    manipulatorInputs[0]);
            // Rotate to angle
            slides.pivotTo(manipulatorInputs[1], telemetry);
            slides.extendTo(manipulatorInputs[2], telemetry);

        }else if((targetRow + 1) * 23.50 < odometry.getYCoordinate() &&
                (targetColumn + 1) * 23.50 >= odometry.getXCoordinate()){ // Bottom right
            // Aimbot with 315˚
            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI * 7 / 4);
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(),
                    manipulatorInputs[0]);
            // Rotate to angle
            slides.pivotTo(manipulatorInputs[1], telemetry);
            slides.extendTo(manipulatorInputs[2], telemetry);
        }

        claw.openClaw();

        // Go back to substation location

    }
}
