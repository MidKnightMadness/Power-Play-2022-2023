// Add import statements as more data is added
// Auxillary (driver hub, control hub, OD, etc...)
package org.firstinspires.ftc.teamcode.highlevel;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.manipulator.DoubleReverse4Bar;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.odometry.TestingOdometryAlgorithm;
import org.firstinspires.ftc.teamcode.drivetrain.*;

// Encoders, Motors
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Servos




// Everything public except for in-class default values!!!!!!!!!
public class Master {
    public static HardwareMap hardwaremap;

    // Robot dimensions
    final static double leftSensorPos = 7.5; // Probably change this to correct notation
    final static double rightSensorPos = 7.5;
    final static double backSensorPos = 7.5;
    final static double robotLength = 15; // Change this to good notation pls
    final static double robotWidth = 15;


    // Driver hub and Vuforia
    public static Telemetry telemetry;
    private BNO055IMU IMU;
    /* Can extract:
    1. Absolute Orientation (Euler Vector, 100Hz) Three axis orientation data based on a 360° sphere

    2. Absolute Orientation (Quaterion, 100Hz) Four point quaternion output for more accurate data manipulation

    3. Angular Velocity Vector (100Hz) Three axis of 'rotation speed' in rad/s

    4. Acceleration Vector (100Hz) Three axis of acceleration (gravity + linear motion) in m/s^2

    5. Magnetic Field Strength Vector (20Hz) Three axis of magnetic field sensing in micro Tesla (uT)

    6. Linear Acceleration Vector (100Hz) Three axis of linear acceleration data (acceleration minus gravity) in m/s^2

    7. Gravity Vector (100Hz) Three axis of gravitational acceleration (minus any movement) in m/s^2

    8. Temperature (1Hz) Ambient temperature in degrees celsius
     */

    // Odometry                             ________
    public static DcMotorEx encoder1; //   |   ==   |
    public static DcMotorEx encoder2; //   ||       |
    public static DcMotorEx encoder3; //   |   ==   | encoder 1 on top, encoder 2 on bottom, encoder3 on left
    //                                      ˜˜˜˜˜˜˜˜
    public static Vector currentPosition;
    public static Vector travel;
    public static Vector orientation;
    public static Vector normalOrientation;

    // Calibration
    public static DistanceSensor leftSensor;
    public static DistanceSensor rightSensor;
    public static DistanceSensor backSensor;

    // Used to update encoder deltas
    public static double encoder1Reading;
    public static double encoder2Reading;
    public static double encoder3Reading;

    private static final double [] DEFAULT_POSITION = {0, 0}; // Get actual robot starting coordinates in inches on Friday, bottom left relative to our starting side is origin
    // Probably write calibration method w/ tape and obj recognition
    public static Vector STARTING_POSITION;


    // Manipulator
    GridSystem grid;
    public static Turntable turntable;
    public static LinearSlides manipulator1;
    public static DoubleReverse4Bar manipulator2;
    public static double turntableAngle; // Radians, as always
    public static TestingOdometryAlgorithm odometryAlg; // Add this


    // Constructor to fully instantiate robot
    public Master(){
        STARTING_POSITION = new Vector(DEFAULT_POSITION);



        odometryAlg = new TestingOdometryAlgorithm(STARTING_POSITION);

        manipulator1 = new LinearSlides();
        manipulator2 = new DoubleReverse4Bar(hardwaremap);

        leftSensor = hardwaremap.get(DistanceSensor.class, "left calibration distance sensor");
        rightSensor = hardwaremap.get(DistanceSensor.class, "right calibration distance sensor");
        backSensor = hardwaremap.get(DistanceSensor.class, "back calibration distance sensor");
    }

    public static double invSqrt(double x) { // Use this for inverse square root ig, gotta tell judges we used some innovative bit shift algorithm originally in C++ or smth
        double xhalf = 0.5d * x;
        long i = Double.doubleToLongBits(x);
        i = 0x5fe6ec85e7de30daL - (i >> 1);
        x = Double.longBitsToDouble(i);
        x *= (1.5d - xhalf * x * x);
        return x;
    }

    public static Vector calibratePosition(double currentOrientation, Vector currentPosition) {
        double leftDistance = leftSensor.getDistance(DistanceUnit.INCH);
        double rightDistance = rightSensor.getDistance(DistanceUnit.INCH);
        double backDistance = backSensor.getDistance(DistanceUnit.INCH);

        double x = -1; double y = -1;

        if (currentOrientation > Math.PI/2 && currentOrientation < Math.PI) {
            // if the back sensor reading is less than 24
            if (backDistance < 24) {
                // if back sensor is pointing to bottom wall
                if ((backDistance + 7.5) * Math.sin(currentOrientation - Math.PI/2) < 144 - currentPosition.get()[0])
                    y = (backDistance + 7.5) * Math.cos(currentOrientation - Math.PI/2);
                    // if back sensor is pointing to right wall
                else
                    x = 144 - (backDistance + 7.5) * Math.cos(Math.PI-currentOrientation);
            }
            // if right sensor reading is less than 24
            if (rightDistance < 24) {
                // if right sensor is pointing to the right wall
                if (x == -1 && (rightDistance + 7.5) * Math.sin(currentOrientation - Math.PI / 2) < 144 - currentPosition.get()[1])
                    x = 144 - (rightDistance + 7.5) * Math.cos(currentOrientation - Math.PI / 2);
                else if (y == -1)
                    y = 144 - (rightDistance + 7.5) * Math.cos(Math.PI - currentOrientation);
            }
            // if left sensor reading is less than 24
            if (leftDistance < 24) {
                //if left sensor is pointing to bottom wall
                if (y == -1 && (leftDistance + 7.5) * Math.sin(Math.PI - currentOrientation) < currentPosition.get()[0])
                    y = (leftDistance + 7.5) * Math.cos(Math.PI - currentOrientation);
                else if (x == -1)
                    x = (leftDistance + 7.5) * Math.cos(currentOrientation - Math.PI/2);
            }
        }
        else if (currentOrientation > Math.PI && currentOrientation < Math.PI * 3.0/2) {
            // if back sensor reading is less than 24
            if (backDistance < 24) {
                // if back sensor is pointing to right wall
                if ((backDistance + 7.5) * Math.sin(currentOrientation - Math.PI) < 144 - currentPosition.get()[1])
                    x = 144 - (backDistance + 7.5) * Math.cos(currentOrientation - Math.PI);
                    // if back sensor is pointing to top wall
                else
                    y = 144 - (backDistance + 7.5) * Math.cos(Math.PI * 3.0/2 - currentOrientation);
            }
            //right sensor reading is less than 24
            if (rightDistance < 24) {
                // if right sensor pointing to top wall
                if (y == -1 && (rightDistance + 7.5) * Math.sin(currentOrientation - Math.PI) < currentPosition.get()[0])
                    y = 144 - (rightDistance + 7.5) * Math.cos(currentOrientation - Math.PI);
                    // if right sensor is pointing to left wall
                else if (x == -1)
                    x = (rightDistance + 7.5) * Math.cos(Math.PI * 3.0/2 - currentOrientation);
            }
            // if left sensor reading is less than 24
            if (leftDistance < 24) {
                // if left sensor is pointing to right wall
                if (x == -1 && (leftDistance + 7.5) * Math.sin(Math.PI * 3.0/2 - currentOrientation) < currentPosition.get()[1])
                    x = 144 - (leftDistance + 7.5) * Math.cos(Math.PI * 3.0/2 - currentOrientation);
                    // if left sensor is pointing to bottom wall
                else if (y == -1)
                    y = (leftDistance + 7.5) * Math.cos(currentOrientation - Math.PI);
            }
        }
        else if (currentOrientation > Math.PI * 3.0/2 && currentOrientation < Math.PI * 2) {
            // if back sensor reading is less than 24
            if (backDistance < 24) {
                // if back sensor is pointing to top wall
                if ((backDistance + 7.5) * Math.sin(currentOrientation - Math.PI * 3.0 / 2) < currentPosition.get()[0])
                    y = 144 - (backDistance + 7.5) * Math.cos(currentOrientation - Math.PI * 3.0 / 2);
                    // if back sensor is pointing to left wall
                else
                    x = (backDistance + 7.5) * Math.cos(2*Math.PI - currentOrientation);
            }
            // right distance reading is less than 24
            if (rightDistance < 24) {
                // if right sensor is pointing to left wall
                if (x == -1 && (rightDistance + 7.5) * Math.sin(currentOrientation - Math.PI * 3.0/2) < currentPosition.get()[1])
                    x = (rightDistance + 7.5) * Math.cos(currentOrientation - Math.PI);
                    // if right sensor is pointing to bottom wall
                else if (y == -1) {
                    y = (rightDistance + 7.5) * Math.cos(2*Math.PI - currentOrientation);
                }
            }
            // if left sensor reading is less than 24
            if (leftDistance < 24) {
                // if left sensor is pointing to top wall
                if ( y == -1 && (leftDistance + 7.5) * Math.sin(2*Math.PI - currentOrientation) < 144 - currentPosition.get()[0])
                    y = 144 - (leftDistance + 7.5) * Math.cos(2*Math.PI - currentOrientation);
                    // if left sensor is pointing to right wall
                else if (x == -1) {
                    x = 144 - (leftDistance + 7.5) * Math.cos(currentOrientation - Math.PI * 3.0/2);
                }
            }
        }
        else {
            // if back distance reading is less than 24
            if (backDistance < 24) {
                // if back sensor is pointing to left wall
                if ((backDistance + 7.5) * Math.sin(currentOrientation) < currentPosition.get()[1])
                    x = (backDistance + 7.5) * Math.cos(currentOrientation);
                    // if back sensor is pointing to bottom wall
                else
                    y = (backDistance + 7.5) * Math.cos(Math.PI / 2 - currentOrientation);
            }
            // if right distance reading is less than 24
            if (rightDistance < 24) {
                // if right sensor is pointing to bottom wall
                if (y == -1 && (rightDistance + 7.5) * Math.sin(currentOrientation) < 144 - currentPosition.get()[0])
                    y = (rightDistance + 7.5) * Math.cos(currentOrientation);
                    // if right sensor is pointing to right wall
                else
                    x = 144 - (rightDistance) * Math.cos(Math.PI/2 - currentOrientation);
            }

            // if left sensor reading is less than 24
            if (leftDistance < 24) {
                // if left sensor is pointing to left wall
                if (x == -1 && (leftDistance + 7.5) * Math.sin(Math.PI/2 - currentOrientation) < 144 - currentPosition.get()[1])
                    x = (backDistance + 7.5) * Math.cos(Math.PI/2 - currentOrientation);
                // if left sensor is pointing to top wall
                if (y == -1) {
                    y = 144 - (leftDistance + 7.5) * Math.cos(currentOrientation);
                }
            }
        }

        return new Vector(new double[] {x, y});
    }

}
