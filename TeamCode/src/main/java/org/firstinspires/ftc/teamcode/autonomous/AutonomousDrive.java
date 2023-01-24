package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

public class AutonomousDrive {
    public DcMotorEx FRMotor;
    public DcMotorEx FLMotor;
    public DcMotorEx BRMotor;
    public DcMotorEx BLMotor;

    PIDController controllerX;
    PIDController controllerY;
    PIDController controllerRotation;
    PIDCoefficients pidCoefficients = new PIDCoefficients(0.5, 0.14, 0.3, 0.25);

    void initHardware(HardwareMap hardwareMap) {
        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");

        FRMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0.5, 0.5, 0.5, 0.6));
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

    public void init(HardwareMap hardwareMap) {
        initHardware(hardwareMap);

        controllerX = new PIDController(pidCoefficients);
        controllerY = new PIDController(pidCoefficients);
        controllerRotation = new PIDController(pidCoefficients);
    }

    public void setPowers(double fr, double fl, double br, double bl) {
        FRMotor.setPower(fr);
        FLMotor.setPower(fl);
        BRMotor.setPower(br);
        BLMotor.setPower(bl);
    }

    public void goToPose(Vector2 currentPosition, Vector2 target, double currentRotation, double targetRotation, double deltaTime) {
        Vector2 error = target.minus(currentPosition);

        Vector2 direction = error.getNormalized();
        double errorMagnitude = error.getMagnitude();

        double changeX = controllerX.calculate(target.x, currentPosition.x, deltaTime);
        double changeY = controllerY.calculate(target.y, currentPosition.y, deltaTime);
        double changeRotation = controllerRotation.calculate(targetRotation, currentRotation, deltaTime);

        double averageRotation = currentRotation + changeRotation / 2.0;

        double cos = Math.cos(averageRotation);
        double sin = Math.sin(averageRotation);

        // cos 1, sin 0
        double correctedX = changeX * cos + changeY * sin;
        double correctedY = changeX * sin + changeY * cos;

        drive(correctedX, correctedY, changeRotation);

    }

    public void drive(double x, double y, double rotate) {
        FRMotor.setPower( x - y + rotate);
        FLMotor.setPower(-x - y - rotate);
        BRMotor.setPower(-x - y + rotate);
        BLMotor.setPower( x - y - rotate);
    }


}
