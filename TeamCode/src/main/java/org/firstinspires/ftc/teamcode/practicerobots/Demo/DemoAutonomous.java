package org.firstinspires.ftc.teamcode.practicerobots.Demo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous
public class DemoAutonomous extends LinearOpMode { //uses LinearOpMode for autonomous
    SampleDrive drive;
    Servo servo;
    Intake intake;
    Lift lift;

    private DistanceSensor sensorDistance;
    private ModernRoboticsI2cRangeSensor sensorRange;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        drive = new SampleDrive(hardwareMap);
        servo = new Servo(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        sensorRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        imu.initialize(parameters);

        sleep(1000); //waits 1 second

        waitForStart(); //waits for start button to be pressed


        // code code code code code code
    }

}