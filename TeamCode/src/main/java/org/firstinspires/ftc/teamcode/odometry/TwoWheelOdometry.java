package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

interface OdometryVariablesab {
    double wheelRadius = 2.1232;
    double wheelCircumference = wheelRadius * Math.PI * 2;

    int ticksPerRotation = 8192;
    double inPerTick = wheelCircumference / ticksPerRotation;

    Vector2 leftWheeelPosition = new Vector2(-3, 0);
    Vector2 rightWheelPosition = new Vector2(3, 0);
    Vector2 topWheelPosition = new Vector2(0, 5);

    double lateralWheelDistance = 10;
    long sleepTime = 100;
}

public class TwoWheelOdometry implements OdometryVariablesab {
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
    DcMotorEx rightEncoder;

    public TwoWheelOdometry(HardwareMap hardwareMap) {
        elapsedTime = new ElapsedTime();
        leftEncoder = hardwareMap.get(DcMotorEx.class, "BR");
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        rightEncoder = hardwareMap.get(DcMotorEx.class, "BL");
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        updateTime();
        updatePosition();

        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    public void updateTime() {
        double currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) / 1000000.0d;
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;
    }

    int leftTicks;
    int rightTicks;
    int topTicks;

    double deltaRadians;
    double forwardMovement;
    double lateralMovementAdjustor;
    double trueLateralMovement;

    double sin;
    double cosine;
    double netX;
    double netY;

    public void updatePosition() {
        leftTicks = leftEncoder.getCurrentPosition();
        rightTicks = rightEncoder.getCurrentPosition();
        topTicks = 0;

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
        deltaRadians = getDeltaRotation(leftDistanceMoved, rightDistanceMoved);
        rotationRadians += deltaRadians;

        // true movement
        forwardMovement = (leftDistanceMoved + rightDistanceMoved) / 2.0d;

        lateralMovementAdjustor = deltaRadians * topWheelPosition.y;
        trueLateralMovement = 0;

        sin = Math.sin(rotationRadians);
        cosine = Math.cos(rotationRadians);

        netX = forwardMovement * cosine + trueLateralMovement * sin;
        netY = forwardMovement * sin + trueLateralMovement * cosine;


        position.x += netX;
        position.y += netY;

        velocity.x = netX / deltaTime;
        velocity.y = netY / deltaTime;
    }

    double getDeltaRotation(double leftChange, double rightChange) {
        return (rightChange - leftChange) / lateralWheelDistance;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("\nTWO WHEEL ODOMETRY");

        telemetry.addLine(String.valueOf(deltaTime));

        telemetry.addData("Wheel ticks", String.format("%d, %d", leftTicks, rightTicks));
        telemetry.addData("Delta wheel ticks", String.format("%d, %d", deltaLeftTicks, deltaRightTicks));

        telemetry.addData("Movement", String.format("%f, %f", forwardMovement, trueLateralMovement));
        telemetry.addData("Net movement", String.format("%d, %d", deltaLeftTicks, deltaRightTicks));

        telemetry.addLine("Position " + position.toString());
        telemetry.addLine("Velocity " + velocity.toString());
        telemetry.addLine(String.format("Rotation: %f", deltaRadians * 180 / Math.PI));
        telemetry.addLine("Rotation " + (rotationRadians * 180 / Math.PI));

    }
}

