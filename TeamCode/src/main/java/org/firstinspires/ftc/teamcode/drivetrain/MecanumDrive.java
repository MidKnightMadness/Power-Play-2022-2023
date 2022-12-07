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

    // Control smoothing
    static final double latencyForSmoothing = 0.5;
    double [] smoothControlsRecursively(double previousX, double previousY, double xInput, double yInput){
        double [] newControls = {0.0, 0.0};

        newControls[0] = previousX * latencyForSmoothing + xInput * (1 - latencyForSmoothing);
        newControls[1] = previousY * latencyForSmoothing + yInput * (1 - latencyForSmoothing);

        return newControls;
    }


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

        // For vector drive testing
//        FRMotor.setDirection(DcMotor.Direction.FORWARD);
//        FLMotor.setDirection(DcMotor.Direction.FORWARD);
//        BRMotor.setDirection(DcMotor.Direction.FORWARD);
//        BLMotor.setDirection(DcMotor.Direction.FORWARD);

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

//    public static double sensitivity = 5.0; // "Steepness" of gradient vectors
//    public double [] correct(){
//        odometryAlg.updateOrientationAndLocation();
//        displacement[0] = -sensitivity * ((currentPosition[0] % 23.50) - 11.75);
//        displacement[1] = -sensitivity * ((currentPosition[1] % 23.50) - 11.75);
//
//        return Vector.add(translation, displacement);
//    }

    public boolean driveToOdometryAlg(double targetX, double targetY, double targetAngle, double currentX, double currentY, double currentAngle, Telemetry telemetry){ // Probably run this every few ticks

//        if(currentX < 5){
//            fieldOrientatedDrive(0, 0.25, 0);

        telemetry.addLine(String.format("replacement = Math.max(%3.2f, %3.2f)", (targetX - currentX) * (targetX - currentX), (targetY - currentY) * (targetY - currentY)));
        telemetry.addLine(String.format("replacement = %3.2f", replacement));
        telemetry.addLine(String.format("fieldOrientatedDrive(%3.2f, %3.2f, %3.2f)", (targetX - currentX) / replacement, (targetY - currentY) / replacement, (targetAngle - currentAngle) / 360));

        if(((targetX - currentX) * (targetX - currentX)) + ((targetY - currentY) * (targetY - currentY)) > 1 &&
            (targetAngle - currentAngle) * (targetAngle - currentAngle) > 10) {

            replacement = Math.max(Math.abs(targetX - currentX), Math.abs(targetY - currentY));


            fieldOrientatedDrive(((targetX - currentX) / replacement), ((targetY - currentY) / replacement), ((targetAngle-currentAngle) / 360)); // 0 on rotational component is temporary, needs correction

//            FRMotor.setPower(-0 + -((targetY - currentY) / replacement) * 0.1 - 0);
//            FLMotor.setPower( 0 + -((targetY - currentY) / replacement) * 0.1 + 0);
//            BRMotor.setPower( 0 + -((targetY - currentY) / replacement) * 0.1 - 0);
//            BLMotor.setPower(-0 + -((targetY - currentY) / replacement) * 0.1 + 0);
            //((targetX - currentX) / replacement) * 0.1
            return false;
        }
        fieldOrientatedDrive(0, 0, 0);
        return true;
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

    public void setPosition(int x, int y, int rotate) {
        FRMotor.setTargetPosition(-x + y - rotate + FRMotor.getCurrentPosition());
        FLMotor.setTargetPosition( x + y + rotate + FLMotor.getCurrentPosition());
        BRMotor.setTargetPosition( x + y - rotate + BRMotor.getCurrentPosition());
        BLMotor.setTargetPosition(-x + y + rotate + BLMotor.getCurrentPosition());

        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FRMotor.setPower(0.5);
        FLMotor.setPower(0.5);
        BRMotor.setPower(0.5);
        BLMotor.setPower(0.5);

        while(FRMotor.isBusy() && FLMotor.isBusy() && BRMotor.isBusy() && BLMotor.isBusy()) {}
    }

    @Deprecated
    public static void setVelocities(DcMotorEx [] motors, double [] vector){
        for(int i = 0; i < vector.length; i++){
            motors[i].setVelocity(vector[i] * MAX);
        }
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
