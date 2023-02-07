package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.highlevel.GridSystem;
import org.firstinspires.ftc.teamcode.manipulator.Claw;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

/*
 * Control Hub Configurations
 * Motors:f
 * 0    FL      REV Robotics 20:1 HD Hex Motor      Left encoder
 * 1    BR      REV Robotics 20:1 HD Hex Motor      Horizontal encoder
 * 2    BL      REV Robotics 20:1 HD Hex Motor
 * 3    FR      REV Robotics 20:1 HD Hex Motor      Right encoder
 *
 * Expansion Hub Configurations
 * Motors:
 * 0    SSM     REV Robotics 20:1 HD Hex Motor
 * 1    LSEM    NeveRest 20 Gearmotor
 * 2    LSEM2   NeveRest 20 Gearmotor
 * Servos:
 * 0    CP    Servo
 * 1    Claw      Servo
                     * 2    CP2     Servo
 */

/*
 * Controls
 * Player 1:
 * left_stick_x     hold        strafe
 * left_stick_y     hold        forward/backward
 * right_stick_x    hold        turn
 * left_bumper      toggle      drive mode
 * right_trigger    hold        faster movement
 * right_bumper     toggle      claw (drops cone)
 *
 * Player 2:
 * left_stick_x     hold        turn
 * left_stick_y     hold        pivot arm
 * right_stick_y    hold        extend linear slides
 * right_bumper     toggle      claw (grabs cone)
 * triangle/y       hold        high junction preset
 * circle/b         hold        middle junction preset
 * cross/a          hold        pick up cone preset
 * dpad_up          hold        pivot claw
 * dpad_down        hold        pivot claw
 */


@TeleOp(name="Mainඞඞඞඞඞඞඞඞඞඞඞඞ")
public class MainTeleOp extends OpMode {
    MecanumDrive mecanum;
    Odometry odometry;
    LinearSlides slides;
    Claw claw;

    public static double [] currentPosition = {0.0, 0.0};

    private boolean lastPressedDriveMode = false;
    private boolean driveModeToggle = false;
    private boolean lastPressedClawOpen = false;
    private boolean clawOpenToggle = false;

    double lastInputY;
    double lastInputX;

    double drivePreviousInputWeight;


    @Override
    public void init() {
        mecanum = new MecanumDrive(hardwareMap);
        odometry = new Odometry(hardwareMap, Math.PI / 2, new Vector2(7.5, 7.5));
        claw = new Claw(hardwareMap);
        slides = new LinearSlides(hardwareMap, 0);//41 * Math.PI / 180);
    }

    double time;
    double deltaTime;
    double previousInputWeight = 0.5;
    final double staticPowerMultiplier = 0.3;
    double powerMultiplier = staticPowerMultiplier;
    double manualC = -1;

    @Override
    public void loop() {
        drive();
        manipulate();
//        test();

        odometry.updatePosition();
        slides.update();
        telemetry();
        telemetry.update();
    }


    void drive() {

        // DRIVER ASSIST
//        if (gamepad1.x || gamepad1.square) {
//            odometry.resetEncoders();
//        }

        powerMultiplier = staticPowerMultiplier + 0.4 * gamepad1.right_trigger; // speeds the driving as trigger is pressed

        if (gamepad1.left_bumper && !lastPressedDriveMode) {
            driveModeToggle = !driveModeToggle;
        }
        lastPressedDriveMode = gamepad1.left_bumper;

//        if (gamepad1.a || gamepad1.cross) {
//            drivePreviousInputWeight += 0.01;
//            if (drivePreviousInputWeight > 1) {
//                drivePreviousInputWeight = 1;
//            }
//            try {
//                sleep(75);
//            } catch (InterruptedException e) {
//                telemetry.addLine(e.toString());
//            }
//        }
//
//        if (gamepad1.b || gamepad1.circle) {
//            drivePreviousInputWeight -= 0.01;
//            if (drivePreviousInputWeight < 0) {
//                drivePreviousInputWeight = 0;
//            }
//            try {
//                sleep(75);
//            } catch (InterruptedException e) {
//            }
//        }

        double adjustedInputX = gamepad1.left_stick_x * (1 - drivePreviousInputWeight) + lastInputX * drivePreviousInputWeight;
        double adjustedInputY = gamepad1.left_stick_y * (1 - drivePreviousInputWeight) + lastInputY * drivePreviousInputWeight;


        if (driveModeToggle) {
            mecanum.fieldOrientatedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    gamepad1.right_stick_x * powerMultiplier, odometry.getRotationRadians());

        } else {
            mecanum.drive(adjustedInputX * powerMultiplier, -adjustedInputY * powerMultiplier,
                    (gamepad1.right_stick_x + gamepad2.left_stick_x) * powerMultiplier * 0.5); // normal drive
        }

        lastInputX = adjustedInputX;
        lastInputY = adjustedInputY;
    }

    void manipulate() {
        // LINEAR SLIDES
        slides.extendBy(-gamepad2.right_stick_y);
        rotateArm(-gamepad2.left_stick_y);

        // CLAW
        if ((gamepad2.right_bumper || gamepad1.right_bumper) && !lastPressedClawOpen) {
            clawOpenToggle = !clawOpenToggle;
        }
        lastPressedClawOpen = (gamepad2.right_bumper || gamepad1.right_bumper);

        if (clawOpenToggle) {
            claw.openClaw();
        } else {
            claw.closeClaw();
        }

        while (gamepad2.circle) { // preset medium junction
            slides.update();
            rotateArmTo(Math.PI / 180 * 116);
            slides.extendTo(28.4);
            if ((gamepad2.right_bumper || gamepad1.right_bumper) && !lastPressedClawOpen) {
                clawOpenToggle = !clawOpenToggle;
            }
            lastPressedClawOpen = (gamepad2.right_bumper || gamepad1.right_bumper);

            if (clawOpenToggle) {
                claw.openClaw();
            } else {
                claw.closeClaw();
            }
        }

        while (gamepad2.triangle) { // preset high junction
            slides.update();
            rotateArmTo(108 * Math.PI / 180);
            slides.extendTo(38.1);
            if ((gamepad2.right_bumper || gamepad1.right_bumper) && !lastPressedClawOpen) {
                clawOpenToggle = !clawOpenToggle;
            }
            lastPressedClawOpen = (gamepad2.right_bumper || gamepad1.right_bumper);

            if (clawOpenToggle) {
                claw.openClaw();
            } else {
                claw.closeClaw();
            }
        }


        while (gamepad2.cross) { // preset grab cone
            slides.update();
            rotateArmTo(0);
            slides.extendTo(19);
            if ((gamepad2.right_bumper || gamepad1.right_bumper) && !lastPressedClawOpen) {
                clawOpenToggle = !clawOpenToggle;
            }
            lastPressedClawOpen = (gamepad2.right_bumper || gamepad1.right_bumper);

            if (clawOpenToggle) {
                claw.openClaw();
            } else {
                claw.closeClaw();
            }
        }

        while (gamepad2.square) { // preset 45 degree start angle
            slides.update();
            rotateArmTo(45 * Math.PI / 180);
            slides.extendTo(19);
            if ((gamepad2.right_bumper || gamepad1.right_bumper) && !lastPressedClawOpen) {
                clawOpenToggle = !clawOpenToggle;
            }
            lastPressedClawOpen = (gamepad2.right_bumper || gamepad1.right_bumper);

            if (clawOpenToggle) {
                claw.openClaw();
            } else {
                claw.closeClaw();
            }
        }

//         claw pivot
        if (gamepad2.dpad_up) {
            manualC = manualC +.01;
        }
        if (gamepad2.dpad_down) {
            manualC = manualC -.01;
        }
    }

    void test() {

//         Adjusting extension length and angle
        if(gamepad2.square){
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
            rotateArmTo(targetAngle);
            slides.extendTo(targetExtension);
            if ((gamepad2.right_bumper || gamepad1.right_bumper) && !lastPressedClawOpen) {
                clawOpenToggle = !clawOpenToggle;
            }
            lastPressedClawOpen = (gamepad2.right_bumper || gamepad1.right_bumper);

            if (clawOpenToggle) {
                claw.openClaw();
            } else {
                claw.closeClaw();
            }
            slides.update();
        }

//        telemetry.addData("\nController target angle (degrees)", targetAngle * 180 / Math.PI);
//        telemetry.addData("Controller target extension length", targetExtension);
//        telemetry.addData("Adjusting extension length", adjustingExtensionLength);
//
//        telemetry.addData("power", -gamepad2.right_stick_y);
//        telemetry.addData("Left stick y", -gamepad2.left_stick_y);
//
//        telemetry.addData("\nController target angle (degrees)", targetAngle * 180 / Math.PI);
//        telemetry.addData("Controller target extension length", targetExtension);
//        telemetry.addData("Adjusting extension length", adjustingExtensionLength);
//
//        telemetry.addData("\nClaw pivot ticks", clawPivotInput);
//        telemetry.addData("Manual claw control ticks", gamepad2.left_trigger);
//
//        slides.update();
//        telemetry.addData("\nPivot angle (degrees)", slides.seesawAngle * 180 / Math.PI);
//        telemetry.addData("Extended length", slides.seesawExtensionLength);
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

//        odometry.telemetry(telemetry);
        mecanum.telemetry(telemetry);
        slides.telemetry(telemetry);
        claw.telemetry(telemetry);

        telemetry.addLine();
        telemetry.addLine("TIMER");
        telemetry.addLine("DeltaTime " + deltaTime);
        telemetry.addLine("Time" + time);
        telemetry.update();
    }

    void goToPosition(double targetX, double targetY, double targetAngle) {
        boolean atLocation = false;

        while (!atLocation) {
            if(gamepad1.left_trigger >= 0.5){
                atLocation = true;
               break;
            }
            telemetry.addData("At Location", atLocation);
            telemetry.addLine(String.format("Current Coordinates: (%3.2f, %3.2f, %3.2f)", odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationDegrees()));
            telemetry.addLine(String.format("Target Coordinates: (%3.2f, %3.2f, %3.2f)", targetX, targetY, targetAngle));
            telemetry.addLine(String.format("Target - current: (%3.2f, %3.2f, %3.2f)", targetX - odometry.getXCoordinate(), targetY - odometry.getYCoordinate(), targetAngle - odometry.getRotationDegrees()));
            telemetry.update();

            odometry.updatePosition();

            atLocation = mecanum.driveTo(targetX, targetY, targetAngle, odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians());

        }
        telemetry.addData("At Location", atLocation);
        telemetry.update();

    }

    public static double clawPivotInput = 0.0;

    public void rotateArm(double radians){
        slides.pivotBy(radians);

        // Needs something to get only 0.1, 0.2, 0.3, etc...
        clawPivotInput =  slides.seesawAngle / Math.PI*.7; // -1.0 (undefined position) if at 180˚, 0.0 if at 0˚ (backwards)
        //clawPivotInput += 1; // 0.0 (backwards) if at 180˚, 1.0 (forwards) if at 0˚

        // Servo only takes inputs in intervals larger than a certain value
        //clawPivotInput = (int) (clawPivotInput * 700.0);
        //clawPivotInput /= 1000.0;

        claw.rotateClaw(1-clawPivotInput + manualC);
    }

    public void rotateArmTo(double angle){
        slides.pivotTo(angle);

        // Needs something to get only 0.1, 0.2, 0.3, etc...
        clawPivotInput = slides.seesawAngle / Math.PI; // -1.0 (undefined position) if at 180˚, 0.0 if at 0˚ (backwards)
        //clawPivotInput += 1; // 0.0 (backwards) if at 180˚, 1.0 (forwards) if at 0˚

        // Servo only takes inputs in intervals of 0.1
        //clawPivotInput = (int) (clawPivotInput * 600.0);
        //clawPivotInput /= 1000.0;

        claw.rotateClaw(clawPivotInput + manualC);
        // Upper may be 0.8 ish, NOT 1.0
    }

    // Cycles once to closest tall junction to substation
    public void cycleToClosestTallJunction(){
        // Grab cone
        claw.openClaw();
        slides.pivotTo(0);
        slides.extendTo(34 - (7.5 - 3.5));
        claw.closeClaw();

        // Raise to position
        slides.pivotTo(Math.PI - Math.atan((37.5 - slides.ROOT_HEIGHT) / (7.5 - 3.5)));
        slides.extendTo(Math.sqrt((37.5 - slides.ROOT_HEIGHT) * (37.5 - slides.ROOT_HEIGHT)  + (7.5 - 3.5) * (7.5 - 3.5)));
        while((!slides.seeSawMotor.isBusy() && !slides.extensionMotor.isBusy() && !slides.extensionMotor2.isBusy())){
            claw.openClaw();
        }
    }

    // Cycles once to specified junction idicies
    private static double DEFAULT_SCORING_RADIUS = 21.0;
    public void cycle(int targetRow, int targetColumn){ // Row number, column number
        // Cone pickup
        claw.openClaw();
        claw.closeClaw();

        // Rotate
        goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians());

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
            slides.pivotTo(manipulatorInputs[1]);
            slides.extendTo(manipulatorInputs[2]);


        }else if((targetRow + 1) * 23.50 >= odometry.getYCoordinate() &&
                (targetColumn + 1) * 23.50 < odometry.getXCoordinate()){ // Top left
            // Aimbot with 135˚
            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI * 3 / 4);
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(),
                    manipulatorInputs[0]);
            // Rotate to angle
            slides.pivotTo(manipulatorInputs[1]);
            slides.extendTo(manipulatorInputs[2]);

        }else if((targetRow + 1) * 23.50 < odometry.getYCoordinate() &&
                (targetColumn + 1) * 23.50 < odometry.getXCoordinate()){ // Bottom left
            // Aimbot with 225˚
            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI * 5 / 4);
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(),
                    manipulatorInputs[0]);
            // Rotate to angle
            slides.pivotTo(manipulatorInputs[1]);
            slides.extendTo(manipulatorInputs[2]);

        }else if((targetRow + 1) * 23.50 < odometry.getYCoordinate() &&
                (targetColumn + 1) * 23.50 >= odometry.getXCoordinate()){ // Bottom right
            // Aimbot with 315˚
            manipulatorInputs = GridSystem.pointAtJunction(odometry.getXCoordinate(), odometry.getYCoordinate(), Math.PI * 7 / 4);
            goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(),
                    manipulatorInputs[0]);
            // Rotate to angle
            slides.pivotTo(manipulatorInputs[1]);
            slides.extendTo(manipulatorInputs[2]);
        }

        claw.openClaw();

        // Reset for cone pickup
        slides.extendTo(slides.STARTING_EXTENDER_LENGTH);
        slides.pivotTo(0);
        goToPosition(odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians());

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
}
