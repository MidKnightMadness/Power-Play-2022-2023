package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.highlevel.Master.currentPosition;
import static org.firstinspires.ftc.teamcode.highlevel.Master.odometryAlg;

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
import org.firstinspires.ftc.teamcode.highlevel.Master.*;
import org.firstinspires.ftc.teamcode.Odometry.Odometry;
//import org.firstinspires.ftc.teamcode.odometry.Odometry;

public class MecanumDrive {
    public DcMotorEx FRMotor;
    public DcMotorEx FLMotor;
    public DcMotorEx BRMotor;
    public DcMotorEx BLMotor;

    Odometry odometry;

    // Order for power values: FL, FR, RL, RR
    // Make sure to normalize power values 0 to 1
    public static final double [] NULL = {0.0, 0.0, 0.0, 0.0};

    public static final double [] backwards = {-1.0, 1,0, -1.0, 1.0};
    public Vector BACKWARDS = new Vector(backwards);

    public static final double [] rightVector = {1.0, 1.0, -1.0, -1.0};
    public Vector RIGHT = new Vector(rightVector);

    public static final double [] turnRight = {-1.0, -1.0, -1.0, -1.0};
    public Vector TURN_RIGHT = new Vector(turnRight);

    public DcMotorEx [] MOTORS = {FLMotor, FRMotor, BLMotor, BRMotor};

    // Navigation
    public static final double [] NULL_POSITION = {0.0, 0.0};
    public static final double [] DEFAULT_DIRECTION = {1.0, 0.0}; //Unit direction vector

    public static final double MAX = 3.1416; //Max speed

    private Vector position;
    private Vector velocity;
    private Vector drive;
    private Vector displacement;

    private Vector translation;
    private Vector rotation;

    int time;
    double maxValue;

    private BNO055IMU imu;

    public MecanumDrive(HardwareMap hardwareMap) {
        // Connect Motors
        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");

        // Set Directions
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set Motor Mode
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        //
//        FRMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));
//        FLMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));
//        BRMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));
//        BLMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));


        MOTORS = new DcMotorEx[]{FLMotor, FRMotor, BLMotor, BRMotor};

        velocity = new Vector(NULL_POSITION);
        drive = new Vector(NULL);
        displacement = new Vector(new double [] {0.0, 0.0});
        translation = new Vector(NULL_POSITION);
        rotation = new Vector(NULL_POSITION);

        time = 0; // Ticks ig


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
    }

    public void drive(double x, double y, double rotate) {
        FRMotor.setPower(-x + y - rotate);
        FLMotor.setPower( x + y + rotate);
        BRMotor.setPower( x + y - rotate);
        BLMotor.setPower(-x + y + rotate);
    }

    public void vectorDrive(double x, double y, double rotate, Telemetry telemetry) {
        telemetry.addData("backwards[0]", backwards[0]);
        telemetry.addData("BACKWARDS.get()[0]", BACKWARDS.get()[0]);
//        telemetry.addData("rightVector[0]", rightVector[0]);
        telemetry.addData("RIGHT.get()[0", RIGHT.get()[0]);
        translation = BACKWARDS.multiply(y).add(RIGHT.multiply(x));
        rotation = TURN_RIGHT.multiply(rotate);
        drive = translation.add(rotation);

        double maxValue = 0.0;
        for(double thisNum : drive.get()){
            if(Math.abs(thisNum) > maxValue){
                maxValue = thisNum;
            }
        }

        setPowers(drive.multiply(1.0 / maxValue), telemetry);

        telemetry.addLine("Right: " + (MAX * x));
        telemetry.addLine("Forwards: " + (MAX * -1 * -y));
//        telemetry.addLine(position.get()[0] + ", " + position.get()[1]);
    }

//    public void goToPosition(double targetXPosition, double targetYPosition, double power, double targetOrientation) {
//        double distanceToXTarget = targetXPosition - odometry.getXCoordinate();
//        double distanceToYTarget = targetYPosition - odometry.getYCoordinate();
//
//        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
//
//        while(FRMotor.isBusy() && FLMotor.isBusy() && BRMotor.isBusy() && BLMotor.isBusy()) {
//            distanceToXTarget = targetXPosition - odometry.getXCoordinate();
//            distanceToYTarget = targetYPosition - odometry.getYCoordinate();
//
//            double robotAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
//
//
//        }
//    }

    public void fieldOrientatedDrive(double x, double y, double rotate, Telemetry telemetry) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float pi = 3.1415926f;
        float gyro_degrees = angles.firstAngle;
        float gyro_radians = gyro_degrees * pi / 180;

        y = -y;
        double offAngle = 0;

        if (x != 0) offAngle = Math.atan(y / x);

        if (x <= 0){ // Getting displacement angle
            offAngle = Math.PI - offAngle; // Might wanna use taylor series to approximate atan later since calculation times are gonna be annoying
        }
        double correctedX = Math.cos(-gyro_radians + offAngle);
        double correctedY = Math.sin(-gyro_radians + offAngle);

        correctedX = correctedX;
        correctedY = -correctedY;

        FRMotor.setPower(-correctedX + correctedY - rotate);
        FLMotor.setPower( correctedX + correctedY + rotate);
        BRMotor.setPower( correctedX + correctedY - rotate);
        BLMotor.setPower(-correctedX + correctedY + rotate);

        telemetry.addData("x2", correctedX);
        telemetry.addData("y2", correctedY);
        telemetry.addData("rotate", rotate);
        telemetry.addData("First Angle", angles.firstAngle);
    }

//    public void driveTo(Vector target, Vector currentPosition){ // Probably run this every few ticks
//        displacement = target.add(currentPosition.multiply(-1)); // Normalize this when inputting for ratios
//
//        drive = RIGHT.multiply(displacement.normalize().get()[0])
//                .add(BACKWARDS.multiply(displacement.normalize().get()[1]));
//
//        setPowers(MOTORS, drive);
//    }

    public static double sensitivity = 5.0; // "Steepness" of gradient vectors
    public Vector correct(){
        odometryAlg.updateOrientationAndLocation();
        displacement.set(0, -sensitivity * ((currentPosition.get()[0] % 23.50) - 11.75));
        displacement.set(1, -sensitivity * ((currentPosition.get()[1] % 23.50) - 11.75));

        return translation.add(displacement);
    }

//    public void driveTo(double currentX, double currentY, double targetX, double targetY){ // Probably run this every few ticks
//        displacement.set(0, targetX - currentX);
//        displacement.set(1, targetY - currentY);
//
//        drive = RIGHT.multiply(displacement.normalize().get()[0])
//                .add(BACKWARDS.multiply(displacement.normalize().get()[1]));
//
//        setPowers(MOTORS, drive);
//    }

    @Deprecated
    public static void setVelocities(DcMotorEx [] motors, Vector vector){
        for(int i = 0; i < vector.get().length; i++){
            motors[i].setVelocity(vector.get()[i] * MAX);
        }
    }

    public void setPowers(Vector vector, Telemetry telemetry){
//        for(int i = 0; i < motors.length; i++){
//            motors[i].setPower(vector.get()[i]);
//            telemetry.addData("powa", vector.get()[i]);
//        }

        FLMotor.setPower(vector.get()[0]);
        FRMotor.setPower(vector.get()[1]);
        BLMotor.setPower(vector.get()[2]);
        BRMotor.setPower(vector.get()[3]);
        telemetry.addData("powa", vector.get()[0]);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("FR Motor Position", FRMotor.getCurrentPosition());
        telemetry.addData("FL Motor Position", FLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Position", BRMotor.getCurrentPosition());
        telemetry.addData("BL Motor Position", BLMotor.getCurrentPosition());
    }
}
