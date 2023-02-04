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

    public double distanceToMaxPower = 6;
    public double maxPower = 0.7;

    Telemetry telemetry;

    PIDController controllerX;
    PIDController controllerY;
    PIDController controllerRotation;

    PIDCoefficients pidCoefficientsX;
    PIDCoefficients pidCoefficientsY;
    PIDCoefficients pidCoefficientsRotation;

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
//        pidCoefficientsX = new PIDCoefficients(0.75, 0, 0, 0.1);
//        pidCoefficientsY = new PIDCoefficients(0.75, 0, 0, 0.1);
//        pidCoefficientsRotation = new PIDCoefficients(0.5, 0.0, 0.3, 0.0);

//        controllerX = new PIDController(pidCoefficientsX);
//        controllerY = new PIDController(pidCoefficientsY);
//        controllerRotation = new PIDController(pidCoefficientsRotation);
    }

    void setPID(double maxPower, double distanceToMaxPower, PIDCoefficients pidCoefficientsX, PIDCoefficients pidCoefficientsY, PIDCoefficients pidCoefficientsRotation) {
        this.maxPower = maxPower;
        this.distanceToMaxPower = distanceToMaxPower;
        controllerX = new PIDController(pidCoefficientsX);
        controllerY = new PIDController(pidCoefficientsY);
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

        double cos = Math.cos(currentRotation);
        double sin = Math.sin(currentRotation);

        telemetry.addData("Current Rotation", currentRotation);
        telemetry.addData("Target Rotation", targetRotation);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Target Position", targetRotation);

        // field x and y
        double correctedY = changeX * cos - changeY * sin;
        double correctedX = changeX * sin + changeY * cos;

        Vector2 pidPosition = new Vector2(changeX, changeY);
        telemetry.addData("PID output", pidPosition);
//        Vector2 pidDirection = pidPosition.getNormalized();
//        double pidMagnitude = pidPosition.getMagnitude();

        telemetry.addData("Y powers", Math.min(maxPower, pidPosition.y / distanceToMaxPower));
        telemetry.addData("X power", Math.min(maxPower, pidPosition.x / distanceToMaxPower));

        drive(limitPower(correctedX/ distanceToMaxPower), -limitPower(correctedY / distanceToMaxPower), 0);

    }

    double limitPower(double power) {
        if (power >= 0.0) {
            return Math.min(maxPower, power);
        }
        else {
            return Math.max(-maxPower, power);
        }

    }

    public void drive(double x, double y, double rotate) {
        FRMotor.setPower( x - y + rotate);
        FLMotor.setPower(-x - y - rotate);
        BRMotor.setPower(-x - y + rotate);
        BLMotor.setPower( x - y - rotate);
    }

}
