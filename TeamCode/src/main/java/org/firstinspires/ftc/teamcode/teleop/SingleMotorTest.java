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
    public static int [] bounds = {0, 0};
    public double presetLength = 19.0;
    public static double inchesPerTick = 0.0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.0, 0.0, 0.0, 0.0));
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper){ // Extend to preset
            motor.setTargetPosition((int) ((presetLength - 19.0) / inchesPerTick));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1.0);

        }else{
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(-gamepad1.right_stick_y);
        }

        if(gamepad1.x){ // Set bottom bound
            bounds[0] = motor.getCurrentPosition();
            if(bounds[1] != 0){
                inchesPerTick = (34 - 19) / Math.abs(bounds[0] - bounds[1]);
            }
        }
        if(gamepad1.y){ // Set top bound
            bounds[1] = motor.getCurrentPosition();
            if(bounds[0] != 0){
                inchesPerTick = (34 - 19) / Math.abs(bounds[0] - bounds[1]);
            }
        }

        if(gamepad1.dpad_up){
            presetLength += 0.001;
        }else if(gamepad1.dpad_down){
            presetLength -= 0.001;
        }

        telemetry.addData("Motor position:", motor.getCurrentPosition());
        telemetry.addData("Inches per tick:", inchesPerTick);
        telemetry.addLine(String.format("Bounds: [%4d, %4d]", bounds[0], bounds[1]));
        telemetry.addLine(String.format("Target length: %5.2f inches", presetLength));
    }
}
