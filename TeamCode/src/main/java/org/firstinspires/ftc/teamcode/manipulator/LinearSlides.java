package org.firstinspires.ftc.teamcode.manipulator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



import org.firstinspires.ftc.robotcore.external.Telemetry;
// This comment was pushed
public class LinearSlides {
    private DcMotorEx linearMotor;
    private int motorStartPosition;

    public LinearSlides(HardwareMap hardwareMap) {
        linearMotor = hardwareMap.get(DcMotorEx.class, "linear_motor"); // connect motor
        motorStartPosition = linearMotor.getCurrentPosition();

        linearMotor.setDirection(DcMotor.Direction.FORWARD); // set direction
        //linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        //linearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

        linearMotor.setPower(0);
    }

    public void raise(int targetPosition) {
        linearMotor.setTargetPosition(targetPosition + motorStartPosition);
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearMotor.setPower(1.0);
    }

    public void lower() {
        linearMotor.setTargetPosition(motorStartPosition);
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearMotor.setPower(1.0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Motor Current", linearMotor.getCurrentPosition());
        telemetry.addData("Lift Motor Target", linearMotor.getTargetPosition());
    }
}
