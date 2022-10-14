// Add import statements as more data is added
// Auxillary (driver hub, control hub, OD, etc...)
package org.firstinspires.ftc.teamcode.HighLevel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drivetrain.*;

// Encoders, Motors
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// Servos




// Everything public except for in-class default values!!!!!!!!!
public class Master {
    public static HardwareMap hardwaremap;

    // Controller

    // Driver hub and Vuforia
    public Telemetry telemetry;

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
    public static final Vector STARTING_POSITION = new Vector(DEFAULT_POSITION);


    // Drive motors

    // Manipulator





    // Constructor to fully instantiate robot
    public void initEverything(){

    }
}
