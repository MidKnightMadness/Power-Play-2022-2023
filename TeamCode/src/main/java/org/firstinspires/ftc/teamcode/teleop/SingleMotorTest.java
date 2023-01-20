package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Single Motor Tester")
public class SingleMotorTest extends OpMode {
    DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.0, 0.0, 0.0, 0.0));
        zeroPosition = motor.getCurrentPosition();
    }

    public double lastInput = 0.0;
    public double hysteresisCoefficient = 0.5;
    public double motorInput = 0.0;
    int zeroPosition = 0;

    @Override
    public void loop() {
//        motorInput = lastInput * hysteresisCoefficient + (1 - hysteresisCoefficient) * gamepad1.right_stick_x;
        motor.setPower(gamepad1.right_stick_y);

//        if(motor.getCurrentPosition() > zeroPosition){
//            motor.setPower(gamepad1.right_stick_y);
//        }else{
//            motor.setPower(Math.max(gamepad1.right_stick_y, 0));
//        }
//        lastInput = motorInput;

        if(gamepad1.x){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(gamepad1.dpad_up){
            hysteresisCoefficient -= 0.001;
        }else if(gamepad1.dpad_down){
            hysteresisCoefficient += 0.001;
        }

        telemetry.addData("Motor position:", motor.getCurrentPosition());
        telemetry.addData("Hysteresis coefficient:", hysteresisCoefficient);
    }
}
