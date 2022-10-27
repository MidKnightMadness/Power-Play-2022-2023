// Add import statements as more data is added
// Auxillary (driver hub, control hub, OD, etc...)
package org.firstinspires.ftc.teamcode.highlevel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    // Controller
    public static Gamepad gamepad1;
    public static Gamepad gamepad2;

    // Driver hub and Vuforia
    public static Telemetry telemetry;

    // Odometry                             ________
    public static DcMotorEx encoder1; //   |   ==   |
    public static DcMotorEx encoder2; //   ||       |
    public static DcMotorEx encoder3; //   |   ==   | encoder 1 on top, encoder 2 on bottom, encoder3 on left
    //                                      ˜˜˜˜˜˜˜˜
    public static Vector currentPosition;
    public static Vector travel;
    public static Vector orientation;
    public static Vector normalOrientation;

    // Used to update encoder deltas
    public static double encoder1Reading;
    public static double encoder2Reading;
    public static double encoder3Reading;

    private static final double [] DEFAULT_POSITION = {0, 0}; // Get actual robot starting coordinates in inches on Friday, bottom left relative to our starting side is origin
    // Probably write calibration method w/ tape and obj recognition
    public static Vector STARTING_POSITION;
    public static MecanumDrive drive;

    // Drive motors

    // Manipulator
    GridSystem grid;
    public static Turntable turntable;
    public static LinearSlides manipulator1;
    public static DoubleReverse4Bar manipulator2;
    public static double turntableAngle; // Radians, as always
    public static TestingOdometryAlgorithm odometryAlg; // Add this
    // Constructor to fully instantiate robot
    public static void initEverything(){ // Lets finish this sometime lol
        STARTING_POSITION = new Vector(DEFAULT_POSITION);
        gamepad1 = hardwaremap.get(Gamepad.class, "Gamepad 1");
        gamepad2 = hardwaremap.get(Gamepad.class, "Gamepad 2");

        drive = new MecanumDrive(hardwaremap);
        odometryAlg = new TestingOdometryAlgorithm(STARTING_POSITION);

        manipulator1 = new LinearSlides();
        manipulator2 = new DoubleReverse4Bar(hardwaremap);
    }

    public static double invSqrt(double x) { // Use this for inverse square root ig, gotta tell judges we used some innovative bit shift algorithm originally in C++ or smth
        double xhalf = 0.5d * x;
        long i = Double.doubleToLongBits(x);
        i = 0x5fe6ec85e7de30daL - (i >> 1);
        x = Double.longBitsToDouble(i);
        x *= (1.5d - xhalf * x * x);
        return x;
    }

}
