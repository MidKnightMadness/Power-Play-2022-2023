package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Timer;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import static java.lang.Thread.sleep;

@TeleOp(name = "Test Odometry", group = "Test")
public class OdometryTest extends OpMode{

    Odometry odometry;
    MecanumDrive mecanum;

    @Override
    public void init() {
        timer = new Timer();
        odometry = new Odometry(hardwareMap, Math.PI / 2, new Vector2(0.0, 0.0));
        mecanum = new MecanumDrive(hardwareMap);
    }

    double time;
    double deltaTime;
    Timer timer;

    double drivePreviousInputWeight = 0.95;
    double lastInputX;
    double lastInputY;

    final double staticPowerMultiplier = 0.3;
    double powerMultiplier = staticPowerMultiplier;

    private boolean lastPressedDriveMode = false;
    private boolean driveModeToggle = false;

    void drive() {

        // DRIVER ASSIST
        if (gamepad1.x || gamepad1.square) {
            odometry.resetEncoders();
        }

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
                    gamepad1.right_stick_x * powerMultiplier * 0.5); // normal drive
        }

        lastInputX = adjustedInputX;
        lastInputY = adjustedInputY;
    }

    double ticksPerSecond = 0.0;

    @Override
    public void loop() {
        timer.updateTime();
        time = timer.getTime();
        deltaTime = timer.getDeltaTime();

        drive();

        odometry.updateTime();
        odometry.updatePosition();
        ticksPerSecond = 1.0 / odometry.deltaTime;
        telemetry.addData("Ticks per second", ticksPerSecond);


        telemetry.addLine("PRESS X OR SQUARE to RESET ENCODERS");
        odometry.telemetry(telemetry);
        telemetry.update();

    }

}

