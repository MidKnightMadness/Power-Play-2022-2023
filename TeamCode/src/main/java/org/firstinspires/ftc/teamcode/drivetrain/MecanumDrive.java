package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.highlevel.Master.currentPosition;
import static org.firstinspires.ftc.teamcode.highlevel.Master.invSqrt;
import static org.firstinspires.ftc.teamcode.highlevel.Master.odometryAlg;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.highlevel.Master;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.drivetrain.Vector;
//import org.firstinspires.ftc.teamcode.odometry.Odometry;
import static org.firstinspires.ftc.teamcode.highlevel.Master.telemetry;

public class MecanumDrive {
    public DcMotorEx FRMotor;
    public DcMotorEx FLMotor;
    public DcMotorEx BRMotor;
    public DcMotorEx BLMotor;

    Odometry odometry;

    // Order for power values: FL, FR, RL, RR
    // Make sure to normalize power values 0 to 1

    public static double [] BACKWARDS = {-1.0, 1,0, -1.0, 1.0};

    public static double [] RIGHT = {1.0, 1.0, -1.0, -1.0};

    public static final double [] TURN_RIGHT = {-1.0, -1.0, -1.0, -1.0};

    // Navigation
    public static final double [] NULL_POSITION = {0.0, 0.0};
    public static final double [] DEFAULT_DIRECTION = {1.0, 0.0}; //Unit direction vector

    public static final double MAX = 3.1416; //Max speed

    private double [] position = {0.0, 0.0};
    private double [] velocity = {0.0, 0.0};
    public static double [] drive = {0.0, 0.0, 0.0, 0.0};
    private double [] displacement = {0.0, 0.0};
    private double [] auxillary = {0.0, 0.0};

    public static double [] translation = {0.0, 0.0, 0.0, 0.0};
    public static double [] rotation = {0.0, 0.0, 0.0, 0.0};

    int time;
    double maxValue;

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
//        FRMotor.setDirection(DcMotor.Direction.FORWARD);
//        FLMotor.setDirection(DcMotor.Direction.REVERSE);
//        BRMotor.setDirection(DcMotor.Direction.REVERSE);
//        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set Motor Mode
//        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        drive = new double [4];
        drive[0] = 0.0;
        drive[1] = 0.0;
        drive[2] = 0.0;
        drive[3] = 0.0;


        //
//        FRMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));
//        FLMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));
//        BRMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));
//        BLMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));



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

    public static double [] vectorDrive(double x, double y, double rotate) {

        translation = Vector.add(Vector.multiply(x, RIGHT), Vector.multiply(y, BACKWARDS));
//        rotation = Vector.multiply(rotate, TURN_RIGHT);
        MecanumDrive.drive = Vector.multiply(0.5, translation); // temporary ig


        Vector.multiply(1/Math.max(MecanumDrive.drive[0], Math.max(MecanumDrive.drive[1], Math.max(MecanumDrive.drive[2], MecanumDrive.drive[3]))), MecanumDrive.drive);

        return MecanumDrive.drive;

//        FLMotor.setPower(this.drive[0]);
//        FRMotor.setPower(this.drive[1]);
//        BLMotor.setPower(this.drive[2]);
//        BRMotor.setPower(this.drive[3]);
    }

    public void fieldOrientatedDrive(double x, double y, double rotate) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyro_degrees = angles.firstAngle;
        gyro_radians = gyro_degrees * Math.PI / 180;
        offAngle = Math.atan(y / x);
        if (x == 0 && y == 0) {
            FRMotor.setPower(0 - rotate);
            FLMotor.setPower(rotate);
            BRMotor.setPower(0 - rotate);
            BLMotor.setPower(rotate);
            return;
        }

        if (x < 0) { offAngle = Math.PI + offAngle; }

        correctedX = Math.cos(-gyro_radians + offAngle);
        correctedY = Math.sin(-gyro_radians + offAngle);

        FRMotor.setPower(-correctedX + correctedY - rotate);
        FLMotor.setPower( correctedX + correctedY + rotate);
        BRMotor.setPower( correctedX + correctedY - rotate);
        BLMotor.setPower(-correctedX + correctedY + rotate);
    }


    public static double sensitivity = 5.0; // "Steepness" of gradient vectors
    public double [] correct(){
        odometryAlg.updateOrientationAndLocation();
        displacement[0] = -sensitivity * ((currentPosition[0] % 23.50) - 11.75);
        displacement[1] = -sensitivity * ((currentPosition[1] % 23.50) - 11.75);

        return Vector.add(translation, displacement);
    }


    public boolean driveTo(double targetX, double targetY, double targetAngle){ // Probably run this every few ticks
        if(invSqrt(((targetAngle) * (odometryAlg.orientationAngle)) + ((targetX - currentPosition[0]) * (targetX - currentPosition[0])) + ((targetY - currentPosition[1]) * (targetY - currentPosition[1]))) > 10) {
            fieldOrientatedDrive(targetX - currentPosition[0], targetY - currentPosition[1], targetAngle - odometryAlg.orientationAngle);
            return false;
        }
        return true;
    }


    @Deprecated
    public static void setVelocities(DcMotorEx [] motors, double [] vector){
        for(int i = 0; i < vector.length; i++){
            motors[i].setVelocity(vector[i] * MAX);
        }
    }

    public void setPowers(double [] vector, Telemetry telemetry){
        FLMotor.setPower(vector[0]);
        FRMotor.setPower(vector[1]);
        BLMotor.setPower(vector[2]);
        BRMotor.setPower(vector[3]);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("\nMECANUM WHEELS");
        telemetry.addData("FR Motor Position", FRMotor.getCurrentPosition());
        telemetry.addData("FL Motor Position", FLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Position", BRMotor.getCurrentPosition());
        telemetry.addData("BL Motor Position", BLMotor.getCurrentPosition());
        telemetry.addLine();
//        telemetry.addData("Front Left input:", auxillary[0]);
//        telemetry.addData("Front right input:", auxillary[1]);
//        telemetry.addData("Rear left input:", auxillary[2]);
//        telemetry.addData("Rear right input:", auxillary[3]);
        telemetry.addData("x2", correctedX);
        telemetry.addData("y2", correctedY);
        telemetry.addData("First Angle", angles.firstAngle);
    }

}
