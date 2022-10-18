package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.Odometry.*;

// final variables
@Deprecated
interface OdometryVariables {
    double wheelRadius = 1.417325;
    double wheelCircumference = wheelRadius * Math.PI * 2;

    int ticksPerRotation = 8192;
    double inPerTick = wheelCircumference / ticksPerRotation;

    org.firstinspires.ftc.teamcode.odometry.Vector2 leftWheeelPosition = new org.firstinspires.ftc.teamcode.odometry.Vector2(-3, 0);
    org.firstinspires.ftc.teamcode.odometry.Vector2 rightWheelPosition = new org.firstinspires.ftc.teamcode.odometry.Vector2(3, 0);
    org.firstinspires.ftc.teamcode.odometry.Vector2 topWheelPosition = new org.firstinspires.ftc.teamcode.odometry.Vector2(0, 5);

    double lateralWheelDistance = 10;
    double verticalWheelDistance = 5;

}

public class Odometry implements OdometryVariables {
    double deltaTime = 0;
    double lastTime = 0;

    public org.firstinspires.ftc.teamcode.odometry.Vector2 position = new org.firstinspires.ftc.teamcode.odometry.Vector2();
    public org.firstinspires.ftc.teamcode.odometry.Vector2 velocity = new org.firstinspires.ftc.teamcode.odometry.Vector2();

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

    public Odometry(HardwareMap hardwareMap) {
        elapsedTime = new ElapsedTime();

        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "topEncoder");
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateTime() {
        double currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) / 1000000.0d;
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;
    }

    public void updatePosition() {
        int leftTicks = leftEncoder.getCurrentPosition();
        int rightTicks = rightEncoder.getCurrentPosition();
        int topTicks = horizontalEncoder.getCurrentPosition();

        deltaLeftTicks = leftTicks - lastLeftTicks;
        deltaRightTicks = rightTicks - lastRightTicks;
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
    }

    double getDeltaRotation(double leftChange, double rightChange) {
        return (rightChange - leftChange) / lateralWheelDistance;
    }

    public double getXCoordinate() {
        return position.x;
    }

    public double getYCoordinate() {
        return position.y;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Position " + position.toString());
        telemetry.addLine("Velocity " + velocity.toString());
        telemetry.addLine("Rotation " + (rotationRadians * 180 / Math.PI));
    }

}

