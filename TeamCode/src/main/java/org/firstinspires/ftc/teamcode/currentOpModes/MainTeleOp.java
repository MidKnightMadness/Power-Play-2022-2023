package org.firstinspires.ftc.teamcode.currentOpModes;

import static org.firstinspires.ftc.teamcode.drivetrain.Vector.lengthOf;
import static org.firstinspires.ftc.teamcode.highlevel.Master.aimbotActivated;
import static org.firstinspires.ftc.teamcode.highlevel.Master.currentPosition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Timer;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.highlevel.Master;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.TestingOdometryAlgorithm;
import static org.firstinspires.ftc.teamcode.drivetrain.Vector.neg;
import static org.firstinspires.ftc.teamcode.highlevel.Master.manipulator1;
//import static org.firstinspires.ftc.teamcode.highlevel.Master.tracking;

@TeleOp(name="Main")
public class MainTeleOp extends OpMode {
    MecanumDrive mecanum;
    TestingOdometryAlgorithm odometryAlg;
    LinearSlides lift;
    Turntable turntable;
//    HardwareMap hardwareMap;
    Timer timer;
    double auxillary;
    double auxillary1;
    double [] auxillaryList1;
    double [] auxillaryList2;

    private boolean lastPressedLiftMotor = false;
    private boolean liftMotorToggle = false;
    private boolean lastPressedDriveMode = false;
    private boolean driveModeToggle = false;

    @Override
    public void init() {
        timer = new Timer();
        auxillary = 0.0;
        auxillary1 = 0.0;
        auxillaryList1 = new double [] {0.0, 0.0};
        auxillaryList2 = new double [] {0.0, 0.0, 0.0};

        mecanum = new MecanumDrive(hardwareMap);
//        odometry = new Odometry(hardwareMap);
//        lift = new LinearSlides(hardwareMap);
//        turntable = new Turntable(hardwareMap);

    }

    @Override
    public void loop() {
        // Update tickRate for robot speed, etc...
        Master.tickRate = 1 / (timer.getTime() - auxillary); // auxillary is previous time
        auxillaryList1[0] = currentPosition.getVector()[0] - auxillaryList1[0];
        auxillaryList1[1] = currentPosition.getVector()[1] - auxillaryList1[1];
        Master.robotSpeed = lengthOf(auxillaryList1) / (timer.getTime() - auxillary);


        // DRIVER ASSIST
        if (gamepad1.right_bumper && !lastPressedDriveMode) {
            driveModeToggle = !driveModeToggle;
        }
        if (driveModeToggle) {
            mecanum.fieldOrientatedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad1.dpad_up) { mecanum.fieldOrientatedDrive(0, -1, 0); }
            if (gamepad1.dpad_down) { mecanum.fieldOrientatedDrive(0, 1, 0); }
            if (gamepad1.dpad_right) { mecanum.fieldOrientatedDrive(1, 0, 0); }
            if (gamepad1.dpad_left) { mecanum.fieldOrientatedDrive(-1, 0, 0); }
        } else {
            mecanum.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x); // normal drive
//            mecanum.vectorDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);
            if (gamepad1.dpad_up) { mecanum.drive(0, -1, 0); }
            if (gamepad1.dpad_down) { mecanum.drive(0, 1, 0); }
            if (gamepad1.dpad_right) { mecanum.drive(1, 0, 0); }
            if (gamepad1.dpad_left) { mecanum.drive(-1, 0, 0); }
        }
        lastPressedDriveMode = gamepad1.right_bumper;


        // Checks if aimbot is activated
        if(gamepad2.left_bumper){


        }

        turntable.turnBy(gamepad2.left_stick_x);


        //mecanum.vectorDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_y, telemetry);

        // LIFT (DOUBLE REVERSE 4 BAR)
        /*if (gamepad2.dpad_up && !lastPressedLiftMotor) {
            liftMotorToggle = !liftMotorToggle;
        }
        if (liftMotorToggle) {
            lift.raise();
        } else {
            lift.lower();
        }
        lastPressedLiftMotor = gamepad2.dpad_up;*/

        // TURNTABLE
        /*if (gamepad2.right_stick_x != 0) {
            turntable.turn(gamepad2.right_stick_x);
        }*/


//        odometry.updateTime();
//        odometry.updatePosition();
//
//        // TELEMETRY
//        odometry.telemetry(telemetry);
        if (driveModeToggle) telemetry.addLine("Drive Mode: Field-Orientated Drive");
        else telemetry.addLine("Drive Mode: Normal Drive");
        telemetry.addData("\nLeft Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        mecanum.telemetry(telemetry);
        telemetry.update();


        // Iterative setting of variables for "previous" data
        auxillary = timer.getTime();
        auxillaryList1 = currentPosition.getVector(); // Temporary, sets current position
    }

}
