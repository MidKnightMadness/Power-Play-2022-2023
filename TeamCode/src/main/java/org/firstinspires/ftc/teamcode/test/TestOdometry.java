package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.archive.TestingOdometryAlgorithm.DISTANCE_TO_BACK_WHEEL;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.archive.Vector;

@TeleOp (group = "Test")
@Disabled
@Deprecated
public class TestOdometry extends OpMode {

    //    ––––––––
    //   |   ==   | encoder 1
    //   ||       | encoder 3
    //   |   ==   | encoder 2
    //    ––––––––

    // Chassis dimensional constants
    private final double TRACK_WIDTH = 12.4; // Distance between dead wheels 1 and 2 in inches
    private final double DEAD_WHEEL_RADIUS = 7.500 - 2.098;
    private final double TICKS_PER_ROTATION = 8192;

    // Input / Output, calculations
    private double [] orientation = {0.0, 1.0};
    private double [] normalOrientation = {-1.0, 0.0};
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

    private double [] currentPosition = {0.0, 0.0};
    private double [] travel = {0.0, 0.0};

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
        Vector.rotateBy(orientation, angleChange / 2);
        Vector.rotateBy(normalOrientation, angleChange / 2);


        // Previous iteration - erraneous
//        travel = orientation.multiply(0.5 * encoder1Delta + 0.5 * encoder2Delta).add
//                (normalOrientation.multiply(encoder3Delta * Math.cos(orientationAngle) - encoder3Delta * (angleChange / 2)));

        // Current iteration
        travel = Vector.add(Vector.multiply(0.5 * encoder1Delta + 0.5 * encoder2Delta, orientation),
                            Vector.multiply(encoder3Delta - angleChange * DISTANCE_TO_BACK_WHEEL, normalOrientation));

        orientationAngle += angleChange / 2;
        Vector.rotateBy(orientation, angleChange / 2);
        Vector.rotateBy(normalOrientation, angleChange / 2);

        Vector.add(currentPosition, travel);
    }
}
