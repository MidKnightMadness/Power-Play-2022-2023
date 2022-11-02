package org.firstinspires.ftc.teamcode.odometry;

// final variables
public interface OdometryVariables {
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