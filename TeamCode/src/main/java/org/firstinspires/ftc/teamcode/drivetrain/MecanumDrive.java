package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.highlevel.Master.currentPosition;
import static org.firstinspires.ftc.teamcode.highlevel.Master.invSqrt;
import static org.firstinspires.ftc.teamcode.highlevel.Master.odometryAlg;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousNew;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

/*
 * (Control Hub)
 * Motors:
 * 0    FL      HD Hex Motor 20:1
 * 1    BR      HD Hex Motor 20:1
 * 2    BL      HD Hex Motor 20:1
 * 3    FR      HD Hex Motor 20:1
 */


public class MecanumDrive {
    public DcMotorEx FRMotor;
    public DcMotorEx FLMotor;
    public DcMotorEx BRMotor;
    public DcMotorEx BLMotor;


    private Odometry odometry;

    // Navigation
    public static final double [] NULL_POSITION = {0.0, 0.0};
    public static final double [] DEFAULT_DIRECTION = {1.0, 0.0}; //Unit direction vector

    public static final double MAX = 3.1416; //Max speed

    public BNO055IMU imu;

    private double correctedX;
    private double correctedY;
    private double gyro_degrees;
    private double gyro_radians;
    private double offAngle;

    private Orientation angles;

    public MecanumDrive(HardwareMap hardwareMap) {
//         Connect Motors
        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");

        // Set Directions
//        FRMotor.setDirection(DcMotor.Direction.REVERSE);
//        FLMotor.setDirection(DcMotor.Direction.REVERSE);
//        BRMotor.setDirection(DcMotor.Direction.FORWARD);
//        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set Motor Mode
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set Zero Power Behavior
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stops Motors on INIT
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void drive(double x, double y, double rotate) {
        FRMotor.setPower(-x + y - rotate);
        FLMotor.setPower( x + y + rotate);
        BRMotor.setPower( x + y - rotate);
        BLMotor.setPower(-x + y + rotate);
    }


    private static final double ROTATION_LIMIT = 0.3; // Largest proportion possible for rotation component inputted into drive();
    private double inputModulation = 0.0;
    private static double ROTATION_THRESHOLD = 0.02; // Radians, ~20˚
    private static double DISPLACEMENT_THRESHOLD = 0.5; // Inches

    public void fieldOrientatedDrive(double x, double y, double rotate) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//        gyro_degrees = angles.firstAngle;
        gyro_radians = gyro_degrees * Math.PI / 180;
//        gyro_radians = odometry.getRotationRadians() * Math.PI / 180;
        // Can we move odometry class into mecanumDrive so we don't have 2 odometry classes?

        if(x == 0) { x += 0.001; }
        offAngle = Math.atan(y / x);

        if (x == 0 && y == 0) {
            drive(0, 0, rotate);
            return;
        }

        if (x < 0) { offAngle = Math.PI + offAngle; }

        correctedX = Math.cos(-gyro_radians + offAngle);
        correctedY = Math.sin(-gyro_radians + offAngle);


        // Still need to adjust to prevent overshoots on displacement adjustments
        if(Math.abs(rotate) < 1.0) { // Precise adjustments, to not overshoot on rotation
            inputModulation = (1.0 - (ROTATION_LIMIT * rotate * rotate * rotate)) / Math.max(Math.abs(correctedX), Math.abs(correctedY)); // Allocates non-rotational power to translational movement
            drive(correctedX * inputModulation, correctedY * inputModulation, ROTATION_LIMIT * rotate * rotate * rotate); // To prevent roational overcorrection
        } else {
            inputModulation = (1.0 - (ROTATION_LIMIT * rotate / Math.abs(rotate))) / Math.max(Math.abs(correctedX), Math.abs(correctedY)); // Allocates non-rotational power to translational movement
            drive(correctedX * inputModulation, correctedY * inputModulation, ROTATION_LIMIT * rotate / Math.abs(rotate)); // Would max out at limit
        }
    }

    public boolean driveTo(double targetX, double targetY, double targetAngle, double currentX, double currentY, double currentAngle) {
        if(((targetX - currentX) * (targetX - currentX)) + ((targetY - currentY) * (targetY - currentY)) > 0.01 || // Larger discrepancy than 0.1 in.
                Math.abs(targetAngle - currentAngle) > 0.00872664625997) { // Larger discrepancy than 0.5˚

                fieldOrientatedDrive((targetX - currentX), (targetY - currentY), (targetAngle - currentAngle));
                return false;
            }

        fieldOrientatedDrive(0, 0, 0);
        return true;
    }


    public void pointTo(double x, double y) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyro_degrees = angles.firstAngle;
        gyro_radians = (gyro_degrees * Math.PI / 180);
        double newAngle = Math.acos(x);
        if (y < 0) newAngle = 0-newAngle;
        double rotato = (newAngle-gyro_radians+3.1416)%6.283-3.1416;//reference angle
        if (rotato < -3.1416) rotato += 6.283;
        rotato = Math.cbrt(3.1416/rotato);
        //double amt = rotato-gyro_radians;
        //if (Math.abs(amt) > 3.1416) {//if difference > 180 degrees
            //amt = 0-amt;//flip direction
        //}

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("\nMECANUM WHEELS");
        telemetry.addLine(String.format("Front Motor Power: %f %f", FLMotor.getPower(), FRMotor.getPower()));
        telemetry.addLine(String.format(" Back Motor Power: %f %f", BLMotor.getPower(), BRMotor.getPower()));
        telemetry.addData("Corrected X", correctedX);
        telemetry.addData("Corrected Y", correctedY);
        telemetry.addData("First Angle", angles.firstAngle);
    }


}
