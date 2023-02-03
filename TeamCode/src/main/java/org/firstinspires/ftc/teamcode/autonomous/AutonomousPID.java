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

    PIDCoefficients pidCoefficientsMovement = new PIDCoefficients(0.75, 0.5, 1, 0.5);
    PIDCoefficients pidCoefficientsRotation = new PIDCoefficients(0.5, 0.0, 0.3, 0.0);

    @Override
    public void init() {
        timer = new Timer();
        odometry = new Odometry(hardwareMap, Math.PI / 2, new Vector2(0, 0));
        odometry.resetEncoders();
        odometry.setRotation(Math.PI / 2);

        autonomousDrive = new AutonomousDrive(hardwareMap);
        autonomousDrive.setPID(0.2, 24.0, pidCoefficientsMovement, pidCoefficientsRotation);
    }

    Vector2 targetPos = new Vector2(6, 24);
    @Override
    public void loop() {
        timer.updateTime();
        odometry.updateTime();
        odometry.updatePosition();

        autonomousDrive.setTargetState(telemetry, odometry.position, targetPos, odometry.getRotationRadians(), Math.PI / 2, odometry.getDeltaTime());

        telemetry();

    }

    public void telemetry() {
        telemetry.addData("Target Pos", targetPos);
        telemetry.addData("Current Pos", odometry.position);
    }


}
