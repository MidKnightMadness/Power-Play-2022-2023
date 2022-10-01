package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.Odometry.Vector2;

// final variables
interface OdometryVariables {
    double wheelRadius = 2.1232;
    double wheelCircumference = wheelRadius * Math.PI * 2;

    int ticksPerRotation = 8192;
    double inPerTick = wheelCircumference / ticksPerRotation;

}


@TeleOp
public class Odometry extends OpMode implements OdometryVariables {
    double deltaTime = 0;
    double lastTime = 0;

    Vector2 position = new Vector2();
    Vector2 velocity = new Vector2();

    int lastXTicks = 0;
    int deltaXTicks = 0;
    double xDistanceMoved;

    int lastYTicks = 0;
    int deltaYTicks = 0;
    double yDistanceMoved;

    ElapsedTime elapsedTime;

    DcMotorEx xEncoder;
    DcMotorEx yEncoder;

    @Override
    public void init() {
        elapsedTime = new ElapsedTime();
        xEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        xEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        yEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        yEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        int xTicks = (xEncoder.getCurrentPosition());
        int yTicks = (yEncoder.getCurrentPosition());

        deltaYTicks = lastYTicks - yTicks;
        deltaXTicks = lastXTicks - xTicks;

        lastXTicks = xTicks;
        lastYTicks = yTicks;

        xDistanceMoved = inPerTick * deltaXTicks;
        yDistanceMoved = inPerTick * deltaYTicks;

        position.x += xDistanceMoved;
        position.y += yDistanceMoved;

//        telemetry.addLine(String.format("X encoder: %d", XTicks));
//        telemetry.addLine(String.format("Y encoder: %d", YTicks));
        telemetry.addLine(String.format("%f, %f", xDistanceMoved, yDistanceMoved));
        telemetry.addLine("Position " + position.toString());
    }

    public void getVelocity() {
        velocity.x = xDistanceMoved / deltaTime;
        velocity.y = yDistanceMoved / deltaTime;

        telemetry.addLine("Velocity " + velocity.toString());
    }

}

