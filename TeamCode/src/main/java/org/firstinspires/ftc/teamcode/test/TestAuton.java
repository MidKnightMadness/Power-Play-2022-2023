package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.highlevel.Master.invSqrt;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class TestAuton extends OpMode
{

    MecanumDrive mecanum;
    Odometry odometry;

    @Override
    public void init() {
        mecanum = new MecanumDrive(hardwareMap);
        odometry = new Odometry(hardwareMap);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        goToPosition(12, 12);
    }

    @Override
    public void loop() {

    }

    void goToPosition(int targetX, int targetY) {
        boolean atLocation = false;
        while (!atLocation) {
            atLocation = driveTo(targetX, targetY, 0);
            odometry.updatePosition();
            odometry.updateTime();

            telemetry.addData("At Location", atLocation);
            telemetry.addData("X Coordinate", odometry.getXCoordinate());
            telemetry.addData("Y Coordinate", odometry.getYCoordinate());
            telemetry.addData("X Target", targetX);
            telemetry.addData("Y Target", targetY);
            telemetry.update();
        }

    }

    public boolean driveTo(double targetX, double targetY, double targetAngle) { // Probably run this every few ticks
        if(invSqrt(((targetAngle) * (0)) //odometry.deltaRadians * 180 / Math.PI))
                    + ((targetX - odometry.getXCoordinate()) * (targetX - odometry.getXCoordinate()))
                    + ((targetY - odometry.getYCoordinate()) * (targetY - odometry.getYCoordinate()))) > 1) {
            mecanum.fieldOrientatedDrive(targetX - odometry.getXCoordinate(), targetY - odometry.getYCoordinate(), targetAngle - 0); //odometry.getDeltaRotation());
            return false;

        }

        return true;
    }
}

