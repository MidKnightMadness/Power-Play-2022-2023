package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

// final variables
interface OdometryVariables {
    double wheelRadius = 2.1232;
    double wheelCircumference = wheelRadius * Math.PI * 2;

    int ticksPerRotation = 8192;
    double inPerTick = wheelCircumference / ticksPerRotation;

    Vector2 leftWheeelPosition = new Vector2(-3, 0);
    Vector2 rightWheelPosition = new Vector2(3, 0);
    Vector2 topWheelPosition = new Vector2(0, 5);

    double distanceBetweenWheels = leftWheeelPosition.x - rightWheelPosition.x;

}

@TeleOp
public class Odometry extends OpMode implements OdometryVariables {
    double deltaTime = 0;
    double lastTime = 0;

    public Vector2 position = new Vector2();
    public Vector2 velocity = new Vector2();

    int lastLeftTicks = 0;
    int deltaLeftTicks = 0;
    double leftDistanceMoved;

    int lastRightTicks = 0;
    int deltaRightTicks = 0;
    double rightDistanceMoved;

    int lastTopTicks = 0;
    int deltaTopTicks = 0;
    double topDistanceMoved;

    double rotationRadians;

    ElapsedTime elapsedTime;

    DcMotorEx leftEncoder;
    DcMotorEx horizontalEncoder;
    DcMotorEx rightEncoder;

    @Override
    public void init() {
        elapsedTime = new ElapsedTime();
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "topEncoder");
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        updateTime();
        updatePosition();
        getVelocity();

        telemetry.update();
    }

    public void updateTime() {
        double currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) / 1000000.0d;
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;

//        telemetry.addLine(String.format("Time (seconds): %f", currentTime));
//        telemetry.addLine(String.format("Ticks per second: %f", 1.0d / deltaTime));
    }

    public void updatePosition() {
        int leftTicks = (leftEncoder.getCurrentPosition());
        int rightTicks = (rightEncoder.getCurrentPosition());
        int topTicks = (horizontalEncoder.getCurrentPosition());

        deltaRightTicks = rightTicks - lastRightTicks;
        deltaLeftTicks = leftTicks - lastLeftTicks;
        deltaTopTicks = topTicks - lastTopTicks;

        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;

        leftDistanceMoved = inPerTick * deltaLeftTicks;
        rightDistanceMoved = inPerTick * deltaRightTicks;

        double forwardMovement = (leftDistanceMoved + rightDistanceMoved) / 2.0d;
        double lateralMovement = (topDistanceMoved);

        double angleRadians = getDeltaRotation(leftTicks, rightTicks);

        rotationRadians = angleRadians;

        double sin = Math.sin(rotationRadians);
        double cosine = Math.cos(rotationRadians);
        position.x += forwardMovement * sin + lateralMovement * cosine;
        position.y += forwardMovement * cosine + lateralMovement * sin;

//
//        position.x += rightDistanceMoved;
//        position.y += leftDistanceMoved;

//        telemetry.addLine(String.format("X encoder: %d", XTicks));
//        telemetry.addLine(String.format("Y encoder: %d", YTicks));
//        telemetry.addLine(String.format("%f, %f", xDistanceMoved, yDistanceMoved));
//        telemetry.addLine("Position " + position.toString());
    }

    void getVelocity() {
//        velocity.x = xDistanceMoved / deltaTime;
//        velocity.y = yDistanceMoved / deltaTime;
        telemetry.addLine("Velocity " + velocity.toString());
    }


    double getDeltaRotation(double leftChange, double rightChange) {
        return (leftChange - rightChange) / distanceBetweenWheels;
    }

}

