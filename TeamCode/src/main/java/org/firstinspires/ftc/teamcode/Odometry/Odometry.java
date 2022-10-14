package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

// final variables
@Deprecated
interface OdometryVariables {
    double wheelRadius = 1.417325;
    double wheelCircumference = wheelRadius * Math.PI * 2;

    int ticksPerRotation = 8192;
    double inPerTick = wheelCircumference / ticksPerRotation;

    Vector2 leftWheeelPosition = new Vector2(-3, 0);
    Vector2 rightWheelPosition = new Vector2(3, 0);
    Vector2 topWheelPosition = new Vector2(0, 5);

    double lateralWheelDistance = 10;
    double verticalWheelDistance = 5;

}

// @TeleOp
@Deprecated
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

        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        telemetry.update();
    }

    public void updateTime() {
        double currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) / 1000000.0d;
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;
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
        lastTopTicks = topTicks;

        // raw distance from each encoder
        leftDistanceMoved = inPerTick * deltaLeftTicks;
        rightDistanceMoved = inPerTick * deltaRightTicks;
        topDistanceMoved = inPerTick * deltaTopTicks;

        // angles
        double deltaRadians = getDeltaRotation(leftDistanceMoved, rightDistanceMoved);
        rotationRadians += deltaRadians;

        // true movement
        double forwardMovement = (leftDistanceMoved + rightDistanceMoved) / 2.0d;

        double lateralMovementAdjustor = deltaRadians * verticalWheelDistance;
        double trueLateralMovement = topDistanceMoved - (lateralMovementAdjustor);

        double sin = Math.sin(rotationRadians);
        double cosine = Math.cos(rotationRadians);

        double netX = forwardMovement * cosine + trueLateralMovement * sin;
        double netY = forwardMovement * sin + trueLateralMovement * cosine;

        position.x += netX;
        position.y += netY;

        velocity.x = netX / deltaTime;
        velocity.y = netY / deltaTime;

        telemetry.addLine("Position " + position.toString());
        telemetry.addLine("Velocity " + velocity.toString());
        telemetry.addLine("Rotation " + (rotationRadians * 180 / Math.PI));
    }

    double getDeltaRotation(double leftChange, double rightChange) {
        return (rightChange - leftChange) / lateralWheelDistance;
    }

}

