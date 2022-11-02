package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drivetrain.Vector;

@TeleOp (group = "Test")
public class TestOdometry extends OpMode {

    //    ––––––––
    //   |   ==   | encoder 1
    //   ||       | encoder 3
    //   |   ==   | encoder 2
    //    ––––––––

    // Chassis dimensional constants
    private final double TRACK_WIDTH = 10.0; // Distance between dead wheels 1 and 2 in inches
    private final double DEAD_WHEEL_RADIUS = 1.5;
    private final double TICKS_PER_ROTATION = 8192;

    // Input / Output, calculations
    Vector orientation;
    Vector normalOrientation;
    private final double [] DEFAULT_STARTING_ORIENTATION = {0.0, 1.0};
    private final double [] DEFAULT_STARTING_NORMAL_VECTOR = {-1.0, 0.0};
    private double orientationAngle = Math.PI / 2; // Front-facing angle relative to horizontal at start
    private double angleChange;
    private final double WHEEL_TRAVEL_CONVERSION_FOR_DEAD_WHEELS = Math.PI * 2 * DEAD_WHEEL_RADIUS / TICKS_PER_ROTATION;

    // Encoder deltas in ticks!
    private double encoder1Delta;
    private double encoder2Delta;
    private double encoder3Delta;

    private DcMotorEx encoder1;
    private DcMotorEx encoder2;
    private DcMotorEx encoder3;

    private double encoder1Reading;
    private double encoder2Reading;
    private double encoder3Reading;

    private Vector currentPosition;
    private Vector travel;

    @Override
    public void init() {
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

        currentPosition = new Vector(new double[]{0, 0});
        travel = new Vector(DEFAULT_STARTING_ORIENTATION);
        orientation = new Vector(DEFAULT_STARTING_ORIENTATION);
        normalOrientation = new Vector(DEFAULT_STARTING_NORMAL_VECTOR);

    }

    @Override
    public void loop() {
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

        currentPosition.add(travel);
    }
}
