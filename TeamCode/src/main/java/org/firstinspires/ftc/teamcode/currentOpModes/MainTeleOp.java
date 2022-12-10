package org.firstinspires.ftc.teamcode.currentOpModes;

import static org.firstinspires.ftc.teamcode.drivetrain.Vector.lengthOf;
import static org.firstinspires.ftc.teamcode.highlevel.Master.aimbotActivated;
import static org.firstinspires.ftc.teamcode.highlevel.Master.currentPosition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.highlevel.Master.claw;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Timer;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.highlevel.Master;
import org.firstinspires.ftc.teamcode.manipulator.Claw;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.TestingOdometryAlgorithm;
import static org.firstinspires.ftc.teamcode.drivetrain.Vector.neg;
import static org.firstinspires.ftc.teamcode.highlevel.Master.manipulator1;
import static org.firstinspires.ftc.teamcode.highlevel.Master.turntableAngle;
import static org.firstinspires.ftc.teamcode.manipulator.Turntable.tableMotor;
//import static org.firstinspires.ftc.teamcode.highlevel.Master.tracking;

import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.teamcode.odometry.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

// Control Hub Configurations
// Motors:
// 0 FL
// 1 BR
// 2 BL
// 3 FR

// Expansion Hub Configurations
// Motors:
// 0 SSM
// 1 LSEM
// 2 LSEM2
// Servos
// 0 Claw
// 1 CP
// 2 CP2



@TeleOp(name="Main")
public class MainTeleOp extends OpMode {
    public static MecanumDrive mecanum;
    LinearSlides lift;
    Turntable turntable;
    Claw claw;
    LinearSlides slides;
    public static Odometry odometry;

    public static double [] currentPosition = {0.0, 0.0};

    // First controller left stick
    double[] lastInputs = {0, 0};
    double[] currentInputs = {0, 0};

    Timer timer;

    final double DEADZONE_TOLERANCE = 0.05;

    private boolean lastPressedDriveMode = false;
    private boolean driveModeToggle = false;



    @Override
    public void init() {
        timer = new Timer();

        mecanum = new MecanumDrive(hardwareMap);
        odometry = new Odometry(hardwareMap, Math.PI / 2, new Vector2(0, 0));
        claw = new Claw(hardwareMap);
        slides = new LinearSlides(hardwareMap);
    }

    double time;
    double deltaTime;
    double previousInputWeight = 0.95;
    double powerMultiplier = 0.5;

    @Override
    public void loop() {


        // DRIVER ASSIST
        if (gamepad1.x) {
            odometry.resetEncoders();
        }

        if (gamepad1.a) {
            previousInputWeight += 0.01;
            if (previousInputWeight > 1) { previousInputWeight = 1; }
            try { Thread.sleep(75); }
            catch (InterruptedException e) { telemetry.addLine(e.toString()); }
        }

        if (gamepad1.b) {
            previousInputWeight -= 0.01;
            if (previousInputWeight < 0) { previousInputWeight = 0; }
            try { Thread.sleep(75); }
            catch (InterruptedException e) { }
        }

        currentInputs[0] = gamepad1.left_stick_x * previousInputWeight + lastInputs[0] * (1 - previousInputWeight);
        currentInputs[1] = gamepad1.left_stick_y * previousInputWeight + lastInputs[1] * (1 - previousInputWeight);

        lastInputs[0] = currentInputs[0];
        lastInputs[1] = currentInputs[1];



        // DRIVE
        if (gamepad1.left_bumper && !lastPressedDriveMode) {
            driveModeToggle = !driveModeToggle;
        }

        if (driveModeToggle) {
            mecanum.fieldOrientatedDrive(currentInputs[0] * powerMultiplier, -currentInputs[1] * powerMultiplier,
                    (gamepad1.right_stick_x + gamepad2.left_stick_x) * powerMultiplier);

            if (gamepad1.dpad_up) { mecanum.fieldOrientatedDrive(0, -powerMultiplier, 0); }
            if (gamepad1.dpad_down) { mecanum.fieldOrientatedDrive(0, powerMultiplier, 0); }
            if (gamepad1.dpad_right) { mecanum.fieldOrientatedDrive(powerMultiplier, 0, 0); }
            if (gamepad1.dpad_left) { mecanum.fieldOrientatedDrive(-powerMultiplier, 0, 0); }
        } else {
            mecanum.drive(currentInputs[0] * powerMultiplier, -currentInputs[1] * powerMultiplier,
                    (gamepad1.right_stick_x + gamepad2.left_stick_x) * powerMultiplier); // normal drive

            if (gamepad1.dpad_up) { mecanum.drive(0, -powerMultiplier, 0); }
            if (gamepad1.dpad_down) { mecanum.drive(0, powerMultiplier, 0); }
            if (gamepad1.dpad_right) { mecanum.drive(powerMultiplier, 0, 0); }
            if (gamepad1.dpad_left) { mecanum.drive(-powerMultiplier, 0, 0); }
        }

        mecanum.drive(0, 0, gamepad2.left_stick_x);



        // LINEAR SLIDES
        slides.extendBy(-gamepad2.right_stick_y);



        // SEESAW
        slides.pivotBy(gamepad2.left_stick_y);



        // CLAW
        if (gamepad2.right_bumper && !lastClawOpenToggle) {
            isClawOpenToggle = !isClawOpenToggle;
        }

        lastClawOpenToggle = gamepad2.right_bumper;

        if (isClawOpenToggle) {
            claw.openClaw();
        } else {
            claw.closeClaw();
        }

        // claw pivot
        if (gamepad2.dpad_up) {
            claw.rotateClaw(1);
        } else if (gamepad2.dpad_down) {
            claw.rotateClaw(-1);
        }



        // TELEMETRY
        odometry.updatePosition();

        telemetry.addLine(String.format("Position: [%5.2f, %5.2f]", this.currentPosition[0], currentPosition[1]));
        telemetry.addLine("EASE COEFFICIENT " + previousInputWeight);
        telemetry.addData("Angle", odometry.getRotationRadians() * 180 / Math.PI);


        telemetry.addData("DRIVE MODE", driveModeToggle ? "FIELD ORIENTED": "NORMAL");
        telemetry.addData("Left Stick X", currentInputs[0] * powerMultiplier);
        telemetry.addData("Left Stick Y", currentInputs[0] * powerMultiplier);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);

//        odometry.telemetry(telemetry);
        mecanum.telemetry(telemetry);
        slides.telemetry(telemetry);
        claw.telemetry(telemetry);

        telemetry.addLine("\nTIMER");
        telemetry.addLine("DeltaTime " + deltaTime);
        telemetry.addLine("Time" + time);
        telemetry.update();

    }

    boolean lastClawOpenToggle = false;
    boolean isClawOpenToggle = false;

    void handleManipulatorControls() {
    }

    double deadZone(double input) {
        if (Math.abs(input) < DEADZONE_TOLERANCE) {
            return 0;
        }

        return input;
    }

}
