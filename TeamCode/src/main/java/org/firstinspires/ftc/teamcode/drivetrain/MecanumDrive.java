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

public class MecanumDrive {
    public DcMotorEx FRMotor;
    public DcMotorEx FLMotor;
    public DcMotorEx BRMotor;
    public DcMotorEx BLMotor;


    private Odometry odometry;

    // Temporary:
    public double replacement;

    // Order for power values: FL, FR, RL, RR
    // Make sure to normalize power values 0 to 1

    public static double [] BACKWARDS = {-1.0, 1,0, -1.0, 1.0};
    public static double [] RIGHT = {1.0, 1.0, -1.0, -1.0};
    public static final double [] TURN_RIGHT = {-1.0, -1.0, -1.0, -1.0};

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

        // Se
        //
        // t Zero Power Behavior
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


        replacement = 0.0;
    }

    public void drive(double x, double y, double rotate) {
        FRMotor.setPower(-x + y - rotate);
        FLMotor.setPower( x + y + rotate);
        BRMotor.setPower( x - y - rotate);
        BLMotor.setPower(-x - y + rotate);
    }

    private final double SENSITIVITY = 0.5;
    public void fieldOrientatedDrive(double x, double y, double rotate) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyro_degrees = angles.firstAngle;
        gyro_radians = gyro_degrees * Math.PI / 180;
        offAngle = Math.atan(y / x);

        if (x == 0 && y == 0) {
            drive(0, 0, rotate);
            return;
        }

        if (x < 0) { offAngle = Math.PI + offAngle; }

        correctedX = Math.cos(-gyro_radians + offAngle);
        correctedY = Math.sin(-gyro_radians + offAngle);

        drive(correctedX * SENSITIVITY, correctedY * SENSITIVITY, rotate * SENSITIVITY);
    }

    public boolean driveTo(double targetX, double targetY, double targetAngle){
        if(invSqrt(((targetAngle) * odometry.getRotationDegrees()) +
                      ((targetX - odometry.getXCoordinate()) * (targetX - odometry.getXCoordinate())) +
                      ((targetY - odometry.getYCoordinate()) * (targetY - odometry.getYCoordinate()))) > 10) {
            fieldOrientatedDrive(targetX - odometry.getXCoordinate(), targetY - odometry.getYCoordinate(), targetAngle - odometry.getRotationDegrees());
            return false;
        }
        return true;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("\nMECANUM WHEELS");
        telemetry.addLine(String.format("Front Motor Power: %f %f", FLMotor.getPower(), FRMotor.getPower()));
        telemetry.addLine(String.format(" Back Motor Power: %f %f", BLMotor.getPower(), BRMotor.getPower()));
        telemetry.addData("Left Dead Wheel Position", BRMotor.getCurrentPosition());
        telemetry.addData("Right Dead Wheel Position", BLMotor.getCurrentPosition());
        telemetry.addData("Top Dead Wheel Position", FLMotor.getCurrentPosition());
        telemetry.addData("Corrected X", correctedX);
        telemetry.addData("Corrected Y", correctedY);
        telemetry.addData("First Angle", angles.firstAngle);
    }


}
