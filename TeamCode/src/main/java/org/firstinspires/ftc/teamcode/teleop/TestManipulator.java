package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Timer;

@TeleOp(name = "Manipulator Tester")
public class TestManipulator extends OpMode {
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx pivotMotor;
    
    public static int [] leftBounds = {0, 0};
    public static int [] rightBounds = {0, 0};
    public static int [] pivotBounds = {0, 0};

    public double presetLength = 19.0;
    public static double inchesPerTick = 0.0;
    public static double radiansPerTick = Math.PI / (2 * 1441);


    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "LSEM");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor = hardwareMap.get(DcMotorEx.class, "LSEM2");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotMotor = hardwareMap.get(DcMotorEx.class, "SSM");
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper){ // Extend to preset
            leftMotor.setTargetPosition((int) ((presetLength - 19.0) / inchesPerTick));
            rightMotor.setTargetPosition((int) ((presetLength - 19.0) / inchesPerTick));

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftMotor.setPower(1.0);
            rightMotor.setPower(1.0);

        }else{
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setPower(-gamepad1.right_stick_y);

            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setPower(-gamepad1.right_stick_y);
        }

        if(gamepad1.x){ // Set bottom bound
            leftBounds[0] = leftMotor.getCurrentPosition();
            rightBounds[0] = leftMotor.getCurrentPosition();

            if(leftBounds[1] != 0){
                inchesPerTick = 0.5 * (34 - 19) / Math.abs(leftBounds[0] - leftBounds[1]) + 0.5 * (34 - 19) / Math.abs(rightBounds[0] - rightBounds[1]);
            }
        }
        if(gamepad1.y){ // Set top bound
            leftBounds[1] = leftMotor.getCurrentPosition();
            rightBounds[1] = leftMotor.getCurrentPosition();

            if(leftBounds[0] != 0){
                inchesPerTick = 0.5 * (34 - 19) / Math.abs(leftBounds[0] - leftBounds[1]) + 0.5 * (34 - 19) / Math.abs(rightBounds[0] - rightBounds[1]);
            }
        }

        if(gamepad1.dpad_up){
            presetLength += 0.001;
        }else if(gamepad1.dpad_down){
            presetLength -= 0.001;
        }

        telemetry.addData("Left motor position:", leftMotor.getCurrentPosition());
        telemetry.addData("Right motor position:", rightMotor.getCurrentPosition());
        telemetry.addData("Pivot motor position:", pivotMotor.getCurrentPosition());

        telemetry.addData("\nInches per tick:", inchesPerTick);
        telemetry.addData("Radians per tick:", radiansPerTick);

        telemetry.addData("\nAngle:", pivotMotor.getCurrentPosition() * 180 / (Math.PI * radiansPerTick));
        if(inchesPerTick != 0){
            telemetry.addLine(String.format("\nExtension length: %5.2f in.", (0.5 * leftMotor.getCurrentPosition() * 180 + 0.5 * rightMotor.getCurrentPosition() * 180) / (inchesPerTick)));
        }

        telemetry.addLine(String.format("\nLeft Bounds: [%4d, %4d]", leftBounds[0], leftBounds[1]));
        telemetry.addLine(String.format("Right Bounds: [%4d, %4d]", rightBounds[0], rightBounds[1]));
        telemetry.addLine(String.format("Pivot Bounds: [%4d, %4d]", pivotBounds[0], pivotBounds[1]));

        telemetry.addLine(String.format("\nTarget length: %5.2f inches", presetLength));
    }
}
