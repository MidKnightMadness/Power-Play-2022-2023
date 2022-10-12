package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;

public class MecanumDrive {
    DcMotorEx FRMotor;
    DcMotorEx FLMotor;
    DcMotorEx BRMotor;
    DcMotorEx BLMotor;

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

    Vector position;
    Vector velocity;
    Vector drive;

    Vector translation;
    Vector rotation;

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
        FRMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));
        FLMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));
        BRMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));
        BLMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.00, 0.05, 0.0, 0.0));


        position = new Vector(NULL_POSITION);
        velocity = new Vector(NULL_POSITION);
        drive = new Vector(DEFAULT_DIRECTION);

        time = 0;


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

    public void VectorDrive(double right, double down, double rotateRight, Telemetry telemetry) {
        translation = new Vector(BACKWARDS.multiply(down).add(RIGHT.multiply(right)));
        rotation = new Vector(TURN_RIGHT.multiply(rotateRight));
        drive = translation.add(rotation);

        double maxValue = 0.0;
        for(double thisNum : drive.get()){
            if(Math.abs(thisNum) > maxValue){
                maxValue = thisNum;
            }
        }



        setPowers(MOTORS, drive.multiply(1.0 / maxValue));



        telemetry.addLine("Right: " + (MAX * right) +  "Forwards: " + (MAX * -1 * down));
        telemetry.addLine(position.get()[0] + ", " + position.get()[1]);
        telemetry.update();
    }

    public void drive(double x, double y, double rotation) {
        FRMotor.setPower(-x - y - rotation);
        FLMotor.setPower(-x + y - rotation);
        BRMotor.setPower(-x + y + rotation);
        BLMotor.setPower(-x - y + rotation);
    }


    public static void setVelocities(DcMotorEx [] motors, Vector vector){
        for(int i = 0; i < vector.get().length; i++){
            motors[i].setVelocity(vector.get()[i] * MAX);
        }
    }

    public static void setPowers(DcMotorEx [] motors, Vector vector){
        for(int i = 0; i < vector.get().length; i++){
            motors[i].setPower(vector.get()[i]);
        }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("FR Motor Position", FRMotor.getCurrentPosition());
        telemetry.addData("FL Motor Position", FLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Position", BRMotor.getCurrentPosition());
        telemetry.addData("BL Motor Position", BLMotor.getCurrentPosition());
        telemetry.update();
    }
}
