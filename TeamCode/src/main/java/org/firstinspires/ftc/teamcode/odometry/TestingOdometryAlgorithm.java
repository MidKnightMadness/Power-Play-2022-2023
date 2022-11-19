package org.firstinspires.ftc.teamcode.odometry;

// Auxillary
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.highlevel.Master.STARTING_POSITION;
import static org.firstinspires.ftc.teamcode.highlevel.Master.currentPosition;
import static org.firstinspires.ftc.teamcode.highlevel.Master.encoder1;
import static org.firstinspires.ftc.teamcode.highlevel.Master.encoder2;
import static org.firstinspires.ftc.teamcode.highlevel.Master.encoder3;

import org.firstinspires.ftc.teamcode.drivetrain.*;
import org.firstinspires.ftc.teamcode.highlevel.*;

// Encoders
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestingOdometryAlgorithm {
    private double [] travel = {0.0, 0.0};
    private double [] travel1 = {0.0, 0.0};

    // Use encoder objects encoder1, encorder2, encoder3 from AuxillaryData
    // Encoder wheels are placed as follows:
    //    ________
    //   |   ==   |
    //   ||       |
    //   |   ==   | encoder 1 on top, encoder 2 on bottom, encoder3 on left
    //    ˜˜˜˜˜˜˜˜

    // Chassis dimensional constants
    public static final double TRACK_WIDTH = 12.4; // Distance between dead wheels 1 and 2 in inches
    public static final double DISTANCE_TO_BACK_WHEEL = 7.500 - 2.098; // Distance between center of robot (currentPosition) and back dead wheel - again, in inches
    public static final double DEAD_WHEEL_RADIUS = 2.83464566929 / 2;
    public static final double TICKS_PER_ROTATION = 8192;

    // Input / Output, calculations
    double [] orientation;
    double [] normalOrientation;
    public static final double [] DEFAULT_STARTING_ORIENTATION = {0.0, 1.0};
    public static final double [] DEFAULT_STARTING_NORMAL_VECTOR = {-1.0, 0.0};
    public double orientationAngle = Math.PI / 2; // Front-facing angle relative to horizontal at start
    public double angleChange;
    public static final double WHEEL_TRAVEL_CONVERSION_FOR_DEAD_WHEELS = Math.PI * 2 * DEAD_WHEEL_RADIUS / TICKS_PER_ROTATION;

    // Encoder deltas in ticks!
    private double encoder1Delta;
    private double encoder2Delta;
    private double encoder3Delta;

    // Encoder data
    public int encoder1Reading;
    public int encoder2Reading;
    public int encoder3Reading;


    // Constructor to start everything
    public TestingOdometryAlgorithm(HardwareMap hardwareMap) {
        // Encoders
        encoder1 = hardwareMap.get(DcMotorEx.class, "encoder1");
        encoder2 = hardwareMap.get(DcMotorEx.class, "encoder2");
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
        orientation = DEFAULT_STARTING_ORIENTATION;
        normalOrientation = DEFAULT_STARTING_NORMAL_VECTOR;

    }

    public void updateOrientationAndLocation() {
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
        Vector.equalTo(normalOrientation, Vector.rotateBy(orientation, angleChange / 2));
        Vector.equalTo(normalOrientation, Vector.rotateBy(normalOrientation, angleChange / 2));


        // Previous iteration - erraneous
//        travel = orientation.multiply(0.5 * encoder1Delta + 0.5 * encoder2Delta).add
//                (normalOrientation.multiply(encoder3Delta * Math.cos(orientationAngle) - encoder3Delta * (angleChange / 2)));

        // 2nd iteration, doesn't work due to reference assignment issues
//        travel = Vector.add(Vector.multiply(0.5 * encoder1Delta + 0.5 * encoder2Delta, orientation),
//                Vector.multiply(encoder3Delta - angleChange * DISTANCE_TO_BACK_WHEEL, normalOrientation));

        // Current iteration
        // Component parallel to 2 main wheels, should clear auxillary reference
        travel = Vector.equalTo(travel1, Vector.multiply(0.5 * encoder1Delta + 0.5 * encoder2Delta, orientation));
        // Component parallel to back wheel, should also clear auxillary reference
        travel1 = Vector.equalTo(travel, Vector.multiply(encoder3Delta - angleChange * DISTANCE_TO_BACK_WHEEL, normalOrientation));
        travel = Vector.equalTo(travel, Vector.add(travel, travel1));

        orientationAngle += angleChange / 2;
        Vector.equalTo(normalOrientation, Vector.rotateBy(orientation, angleChange / 2));
        Vector.equalTo(normalOrientation, Vector.rotateBy(normalOrientation, angleChange / 2));

        Vector.equalTo(currentPosition, Vector.add(currentPosition, travel));
    }
}
