package org.firstinspires.ftc.teamcode.manipulator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DoubleReverse4Bar {
    private DcMotorEx liftMotor;
    private int motorStartPosition;

    public DoubleReverse4Bar(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift_motor"); // connect motor
        motorStartPosition = liftMotor.getCurrentPosition();

        liftMotor.setDirection(DcMotor.Direction.FORWARD); // set direction
//        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
//        linearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

        liftMotor.setPower(0);
    }

    public void raise() {
        liftMotor.setTargetPosition(1000 + motorStartPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    public void lower() {
        liftMotor.setTargetPosition(motorStartPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Motor Current", liftMotor.getCurrentPosition());
        telemetry.addData("Lift Motor Target", liftMotor.getTargetPosition());
    }
}
