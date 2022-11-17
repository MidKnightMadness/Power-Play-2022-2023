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
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.TestingOdometryAlgorithm;
import static org.firstinspires.ftc.teamcode.drivetrain.Vector.neg;
import static org.firstinspires.ftc.teamcode.highlevel.Master.manipulator1;
//import static org.firstinspires.ftc.teamcode.highlevel.Master.tracking;

import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;


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

    final double TURNTABLE_DEGREES_PER_SECOND = 180d;
    final double DEADZONE_TOLERANCE = 0.05;
    final double SEEESAW_RADIANS_PER_SECOND = 1;
    final double LINEAR_SLIDER_INCHES_PER_SECOND = 4;

    AndroidAccelerometer accelerometer;

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
//        accelerometer = Master.hardwaremap.get(AndroidAccelerometer.class, "accelerometer");
//        accelerometer.setDistanceUnit(DistanceUnit.INCH);
//        accelerometer.startListening();
    }
    double time = timer.getTime();
    double deltaTime = timer.getDeltaTime();

    @Override
    public void loop() {
        timer.updateTime();
        time = timer.getTime();
        deltaTime = timer.getDeltaTime();

        Master.tickRate = 1 / (time - auxillary); // auxillary is previous time
        auxillaryList1[0] = currentPosition[0] - auxillaryList1[0];
        auxillaryList1[1] = currentPosition[1] - auxillaryList1[1];
        Master.robotSpeed = lengthOf(auxillaryList1) / (time - auxillary);

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
            if (gamepad1.dpad_up) { mecanum.drive(0, -1, 0); }
            if (gamepad1.dpad_down) { mecanum.drive(0, 1, 0); }
            if (gamepad1.dpad_right) { mecanum.drive(1, 0, 0); }
            if (gamepad1.dpad_left) { mecanum.drive(-1, 0, 0); }
        }
        lastPressedDriveMode = gamepad1.right_bumper;

        handleManipulatorControls();

        telemetry.addData("DRIVE MODE: ", driveModeToggle ? "FIELD ORIENTED": "NORMAL");

        telemetry.addData("\nFront Left output:", mecanum.FLMotor.getPower());
        telemetry.addData("Front right output:", mecanum.FLMotor.getPower());
        telemetry.addData("Rear left output:", mecanum.FLMotor.getPower());
        telemetry.addData("Rear right output:", mecanum.FLMotor.getPower());

//        telemetry.addData("\nTime: ", this.timer.elapsedTime);

        telemetry.update();
    }

    boolean lastClawOpenToggle = false;
    boolean isClawOpenToggle = false;

    void handleManipulatorControls() {
         turntable.turnBy(deadZone(this.gamepad2.left_stick_x) * TURNTABLE_DEGREES_PER_SECOND * deltaTime);
         lift.pivotTo( LinearSlides.seesawAngle + deadZone(this.gamepad2.left_stick_y) * SEEESAW_RADIANS_PER_SECOND * deltaTime);
         lift.extendTo(LinearSlides.seesawExtensionLength + deadZone(this.gamepad2.right_stick_x) * LINEAR_SLIDER_INCHES_PER_SECOND * deltaTime );

        if (gamepad2.right_bumper && !lastClawOpenToggle) {
            isClawOpenToggle = !isClawOpenToggle;
        }

        lastClawOpenToggle = gamepad2.right_bumper;

        if (isClawOpenToggle) {
            claw.openClaw();
        }
        else {
            claw.closeClaw();
        }
    }

    double deadZone(double input) {
        if (Math.abs(input) < DEADZONE_TOLERANCE) {
            return 0;
        }

        return input;
    }

}
