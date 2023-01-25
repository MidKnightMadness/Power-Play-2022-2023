package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

public class AutonomousDrive {
    public DcMotorEx FRMotor;
    public DcMotorEx FLMotor;
    public DcMotorEx BRMotor;
    public DcMotorEx BLMotor;

    public double distanceToMaxPower = 24;

    public double maxMovementDistance = 1;
    Telemetry telemetry;

    PIDController controllerX;
    PIDController controllerY;
    PIDController controllerRotation;
    PIDCoefficients pidCoefficientsMovement = new PIDCoefficients(0.5, 0.0, 0.3, 0.0);
    PIDCoefficients pidCoefficientsRotation = new PIDCoefficients(0.5, 0.0, 0.3, 0.0);

    void initHardware(HardwareMap hardwareMap) {
        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");

        // Set Directions
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set Motor Mode
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set Zero Power Behavior
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stops Motors on INIT
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);
    }

    public AutonomousDrive(HardwareMap hardwareMap) {
        initHardware(hardwareMap);

        controllerX = new PIDController(pidCoefficientsMovement);
        controllerY = new PIDController(pidCoefficientsMovement);
        controllerRotation = new PIDController(pidCoefficientsRotation);
    }

    public void setPowers(double fr, double fl, double br, double bl) {
        FRMotor.setPower(fr);
        FLMotor.setPower(fl);
        BRMotor.setPower(br);
        BLMotor.setPower(bl);
    }

    public void setTargetState(Telemetry telemetry, Vector2 currentPosition, Vector2 target, double currentRotation, double targetRotation, double deltaTime) {
        Vector2 errorPosition = target.minus(currentPosition);
        double errorRotation = targetRotation - currentRotation;
        this.telemetry = telemetry;
//        Vector2 errorDirection = errorPosition.getNormalized();
//        double errorMagnitude = errorPosition.getMagnitude();

        // relative to robot x and y
        double changeX = controllerX.calculate(target.x, currentPosition.x, deltaTime);
        double changeY = controllerY.calculate(target.y, currentPosition.y, deltaTime);
        double pidRotation = controllerRotation.calculate(targetRotation, currentRotation, deltaTime);

        double averageRotation = currentRotation + pidRotation / 2.0;

        double cos = Math.cos(averageRotation);
        double sin = Math.sin(averageRotation);

        telemetry.addData("Error Position", errorPosition);
        telemetry.addData("Error Rotation", errorRotation);

        // field x and y
//        double correctedX = changeX * cos + changeY * sin;
//        double correctedY = changeX * sin + changeY * cos;

        Vector2 pidPosition = new Vector2(changeX, changeY);
        telemetry.addData("PID output", pidPosition);
//        Vector2 pidDirection = pidPosition.getNormalized();
//        double pidMagnitude = pidPosition.getMagnitude();

        telemetry.addData("Powers", pidPosition.divide(errorRotation).toString());

        drive(pidPosition.x / distanceToMaxPower, pidPosition.y / distanceToMaxPower, pidRotation / errorRotation);

    }

    public void drive(double x, double y, double rotate) {
        FRMotor.setPower( x - y + rotate);
        FLMotor.setPower(-x - y - rotate);
        BRMotor.setPower(-x - y + rotate);
        BLMotor.setPower( x - y - rotate);
    }

}
