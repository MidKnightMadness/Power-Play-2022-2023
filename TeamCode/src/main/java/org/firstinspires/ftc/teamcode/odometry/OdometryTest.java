package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

interface OdometryVariables {
    double wheelRadius = 1.41546666667;
    double wheelCircumference = wheelRadius * Math.PI * 2;

    int ticksPerRotation = 8192;
    double inPerTick = wheelCircumference / ticksPerRotation;

    double lateralWheelDistance = 12;
    double verticalWheelDistance = 7.5 + 1.4;// 12.4 - 7.5;
    long sleepTime = 100;
}

@TeleOp(name = "Three Wheel Odometry")
public class OdometryTest extends OpMode implements OdometryVariables{
    double deltaTime = 0;
    double lastTime = 0;

    Vector2 position = new Vector2();
    Vector2 velocity = new Vector2();

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

        leftEncoder = hardwareMap.get(DcMotorEx.class, "BR");
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        rightEncoder = hardwareMap.get(DcMotorEx.class, "BL");
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "FL");
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

    public void updatePosition() {
        int leftTicks = leftEncoder.getCurrentPosition();
        int rightTicks = rightEncoder.getCurrentPosition();
        int topTicks = horizontalEncoder.getCurrentPosition();

        telemetry.addData("Wheel ticks", String.format("%d, %d, %d", leftTicks, rightTicks, topTicks));
        telemetry.addData("Delta wheel ticks", String.format("%d, %d, %d", deltaLeftTicks, deltaRightTicks, deltaTopTicks));

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

        telemetry.addData("Movement", String.format("%f, %f", forwardMovement, trueLateralMovement));
        telemetry.addData("Net movement", String.format("%d, %d", deltaLeftTicks, deltaRightTicks));

        position.x += netX;
        position.y += netY;

        velocity.x = netX / deltaTime;
        velocity.y = netY / deltaTime;

        telemetry.addLine("Position " + position);
        telemetry.addLine("Velocity " + velocity.toString());
        telemetry.addLine(String.format("Rotation: %f", deltaRadians * 180 / Math.PI));
        telemetry.addLine("Rotation " + (rotationRadians * 180 / Math.PI));
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

    public String positionToString() {return String.format("(%f, %f)", position.x, position.y); }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Position " + position.toString());
        telemetry.addLine("Velocity " + velocity.toString());
        telemetry.addLine("Rotation " + (rotationRadians * 180 / Math.PI));
    }

}

