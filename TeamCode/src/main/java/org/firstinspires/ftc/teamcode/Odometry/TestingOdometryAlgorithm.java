package org.firstinspires.ftc.teamcode.Odometry;

// Auxillary
import org.firstinspires.ftc.teamcode.Drivetrain.*;
import org.firstinspires.ftc.teamcode.HighLevel.*;

// Encoders
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestingOdometryAlgorithm extends Master {
    HardwareMap hardwareMap;

    DcMotor encoder1;
    DcMotor encoder2;
    DcMotor encoder3;
    // Use encoder objects encoder1, encorder2, encoder3 from Master
    // Encoder wheels are placed as follows:
    //    ________
    //   |   ==   |
    //   ||       |
    //   |   ==   | encoder 1 on top, encoder 2 on bottom, encoder3 on left
    //    ˜˜˜˜˜˜˜˜

    // Chassis dimensional constants
    public static final double TRACK_WIDTH = 10.0; // Distance between dead wheels 1 and 2 in inches
    public static final double DISTANCE_TO_BACK_WHEEL = 4.0; // Distance between center of robot (currentPosition) and back dead wheel - again, in inches
    public static final double DEAD_WHEEL_RADIUS = 1.5;
    public static final double TICKS_PER_ROTATION = 8192;

    // Input / Output, calculations
    public static final double [] DEFAULT_STARTING_ORIENTATION = {0.0, 1.0};
    public static final double [] DEFAULT_STARTING_NORMAL_VECTOR = {-1.0, 0.0};
    public double orientationAngle = Math.PI / 2; // Front-facing angle relative to horizontal at start
    public double angleChange;
    public static final double WHEEL_TRAVEL_CONVERSION_FOR_DEAD_WHEELS = Math.PI * 2 * DEAD_WHEEL_RADIUS / TICKS_PER_ROTATION;

    // Encoder deltas in ticks!
    private double encoder1Delta;
    private double encoder2Delta;
    private double encoder3Delta;

    // Constructor to start everything
    public void initTestingOdometryAlgorithm(Vector startingPosition) {
        // Encoders
        encoder1 = hardwaremap.get(DcMotorEx.class, "encoder1");
        encoder2 = hardwaremap.get(DcMotorEx.class, "encoder2");
        encoder3 = hardwareMap.get(DcMotorEx.class, "encoder3");

        encoder1.setDirection(DcMotor.Direction.REVERSE);
        encoder2.setDirection(DcMotor.Direction.REVERSE);
        encoder3.setDirection(DcMotor.Direction.FORWARD);

        encoder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoder1Reading = encoder1.getCurrentPosition();
        encoder2Reading = encoder2.getCurrentPosition();
        encoder3Reading = encoder3.getCurrentPosition();

        encoder1Delta = 0.0;
        encoder2Delta = 0.0;
        encoder3Delta = 0.0;

        angleChange = 0.0;

        // Vector initialization
        currentPosition = STARTING_POSITION;
        travel = new Vector(DEFAULT_STARTING_ORIENTATION);
        orientation = new Vector(DEFAULT_STARTING_ORIENTATION);
        normalOrientation = new Vector(DEFAULT_STARTING_NORMAL_VECTOR);

    }

    public void updateOrientationAndLocation(){
        // Conversion to inches
        encoder1Delta = (encoder1.getCurrentPosition() - encoder1Reading) * WHEEL_TRAVEL_CONVERSION_FOR_DEAD_WHEELS;
        encoder2Delta = (encoder2.getCurrentPosition() - encoder2Reading) * WHEEL_TRAVEL_CONVERSION_FOR_DEAD_WHEELS;
        encoder3Delta = (encoder3.getCurrentPosition() - encoder3Reading) * WHEEL_TRAVEL_CONVERSION_FOR_DEAD_WHEELS;

        // Tick-updting encoder positions
        encoder1Reading = encoder1.getCurrentPosition();
        encoder2Reading = encoder2.getCurrentPosition();
        encoder3Reading = encoder3.getCurrentPosition();

        // Sandwhich algorithm in 1/2 tick-updating calls for the sake of accuracy
        angleChange = (encoder1Delta - encoder2Delta) / TRACK_WIDTH;

        orientationAngle += angleChange / 2;
        orientation.rotate(angleChange / 2);
        normalOrientation.rotate(angleChange / 2);

        travel = orientation.multiply(0.5 * encoder1Delta + 0.5 * encoder2Delta).add
                (normalOrientation.multiply(encoder3Delta * Math.cos(orientationAngle) - encoder3Delta * (angleChange / 2)));

        orientationAngle += angleChange / 2;
        orientation.rotate(angleChange / 2);
        normalOrientation.rotate(angleChange / 2);
    }
}
