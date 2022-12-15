package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
    private static final double TRANSLATION_LIMIT = 1.0 - ROTATION_LIMIT;
    private double drivenX = 0.0;
    private double drivenY = 0.0;

    public void fieldOrientatedDrive(double x, double y, double rotate, double angle) { // Angle is angle of robot relative to horizontal right, 0 ≤ rotate, x, y ≤ 1

            // Rotation of perpendicular unit drive vectors back to basis vectors
            drivenX = Math.cos(-angle) * x - Math.sin(-angle) * y;
            drivenY = Math.sin(-angle) * x + Math.cos(-angle) * y;

            // Cap values for translation
            drivenX *= TRANSLATION_LIMIT / (Math.abs(drivenX) + Math.abs(drivenY));
            drivenY *= TRANSLATION_LIMIT / (Math.abs(drivenX) + Math.abs(drivenY));

            // Precise adjustments
            if(x < TRANSLATION_LIMIT / 2){
                drivenX /= 2;
            }
            if(y < TRANSLATION_LIMIT / 2){
                drivenY /= 2;
            }

            drive(drivenX, drivenY, ROTATION_LIMIT * rotate / Math.abs(rotate));

    }

    public boolean driveTo(double targetX, double targetY, double targetAngle, double currentX, double currentY, double currentAngle) {
        double dy = (targetY - currentY);
        double dx = (targetX - currentX);
        double rotato = Math.acos(dx / Math.hypot(dy, dx));
        if (dy < 0) rotato = 0 - rotato; //finds angle of approach

        double newx = Math.cos(rotato - currentAngle+Math.PI / 2); //drives without turning to the point
        double newy = Math.sin(rotato - currentAngle+Math.PI / 2);
        double spd = Math.min(Math.hypot(dy, dx), 4.0) / 5;
        spd *= spd * spd; // Cube, still needed more precise adjustment

        if((dx * dx) + (dy * dy) > 0.5 || (targetAngle - currentAngle) * (targetAngle - currentAngle) > 5) {
            drive(newx * spd, newy * spd, -pointTo(targetAngle, currentAngle));
            return false;
        }
        return true;
    }


    public double pointTo(double targetAngle, double currentAngle) { //forward is 0
        double rotato = (targetAngle - currentAngle + 3.1416) % 6.283 - 3.1416; //reference angle
        if (rotato < -3.1416) rotato += 6.283;
        if (Math.abs(rotato) < .087) return rotato / .087 / 4;
        return rotato / Math.abs(rotato) / 4;
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
