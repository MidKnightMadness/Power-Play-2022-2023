package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Timer;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

@Autonomous
public class AutonomousPID extends OpMode {
    public DcMotorEx FRMotor;
    public DcMotorEx FLMotor;
    public DcMotorEx BRMotor;
    public DcMotorEx BLMotor;

    Timer timer;
    Odometry odometry;
    AutonomousDrive autonomousDrive;

    @Override
    public void init() {
        timer = new Timer();
        odometry = new Odometry(hardwareMap, 0, new Vector2(0, 0));
        odometry.resetEncoders();

        autonomousDrive = new AutonomousDrive(hardwareMap);
    }

    @Override
    public void loop() {
        timer.updateTime();
        odometry.updateTime();
        odometry.updatePosition();

        autonomousDrive.setTargetState(telemetry, odometry.position, new Vector2(0, 12), odometry.getRotationRadians(), 0.0, odometry.getDeltaTime());

        telemetry();

    }

    public void telemetry() {
        telemetry.addData("Target Pos", new Vector2(0, 12));
        telemetry.addData("Current Pos", odometry.position);
    }


}
