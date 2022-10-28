package org.firstinspires.ftc.teamcode.robots.carnival;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Bird drive")
public class Drive extends OpMode {
    DcMotorEx FRONT_RIGHT;
    DcMotorEx REAR_RIGHT;
    DcMotorEx FRONT_LEFT;
    DcMotorEx REAR_LEFT;

    DcMotorEx spinWheel;

    @Override
    public void init() {
        FRONT_LEFT = hardwareMap.get(DcMotorEx.class, "fl");
        FRONT_RIGHT = hardwareMap.get(DcMotorEx.class, "fr");
        REAR_RIGHT = hardwareMap.get(DcMotorEx.class, "rr");
        REAR_LEFT = hardwareMap.get(DcMotorEx.class, "rl");

        spinWheel = hardwareMap.get(DcMotorEx.class, "turn");
        spinWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        REAR_LEFT.setDirection(DcMotorSimple.Direction.REVERSE);
        REAR_RIGHT.setDirection(DcMotorSimple.Direction.REVERSE);

        FRONT_LEFT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRONT_RIGHT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        REAR_RIGHT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        REAR_LEFT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void start() {

    }
    float powerLimiter = 0.2f;

    @Override
    public void loop() {
        double stickX = this.gamepad1.right_stick_x;
        double stickY = this.gamepad1.right_stick_y;

        double angle = Math.atan2(stickX, stickY) * 180.0d / Math.PI;
        int direction = (int) Math.round(angle / 45.0d);

        telemetry.addLine("Angle " + (angle));
        telemetry.addLine("Direction " + (direction));


        float power = powerLimiter * (float) (stickX * stickX + stickY * stickY);

        double leftStickX = this.gamepad1.left_stick_x;

        if (leftStickX == 1) {
            SetMotors(-powerLimiter, 0, 0, 0);
        }
        else if (leftStickX == -1) {
            SetMotors(powerLimiter, 0, 0, -0);
        }

        telemetry.addLine(String.valueOf(leftStickX));

        if (this.gamepad1.a) {
            spinWheel.setVelocity(-0.5f);
        }
        else {
            spinWheel.setPower(0);
        }

        if (true) {
            switch (direction) {
                case 0:
                    MoveBack(power);
                    break;
                case 1:
                    MoveBackRight(power);
                    break;
                case 2:
                    MoveRight(power);
                    break;
                case 3:
                    MoveFrontRight(power);
                    break;
                case 4:
                    MoveForward(power);
                    break;
                case -3:
                    MoveFrontLeft(power);
                case -2:
                    MoveLeft(power);
                    break;
                case -1:
                    MoveBackLeft(power);
            }
        }

        telemetry.update();
    }

    void MoveForward (float power) {
        SetMotors(power, power, power, power);
    }

    void MoveLeft(float power) {
        SetMotors(-power, power, -power, power);
    }
    void MoveRight(float power) {
        SetMotors(power, -power, power, -power);
    }
    void MoveBack(float power) {
        SetMotors(-power, -power, -power, -power);
    }

    void MoveFrontRight(float power) {
        SetMotors(power, 0, power, 0);
    }

    void MoveFrontLeft(float power) {
        SetMotors(0, power, 0, power);
    }

    void MoveBackLeft(float power) {
        SetMotors(-power, 0, -power, 0);
    }

    void MoveBackRight(float power) {
        SetMotors(0, -power, 0, -power);
    }



    void SetMotors(float flPower, float frPower, float rlPower, float rrPower) {
        REAR_LEFT.setPower(rlPower);
        REAR_RIGHT.setPower(rrPower);
        FRONT_RIGHT.setPower(frPower);
        FRONT_LEFT.setPower(flPower);
    }
}

