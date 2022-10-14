package org.firstinspires.ftc.teamcode.manipulator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turntable {
    private DcMotorEx tableMotor;

    public Turntable(HardwareMap hardwareMap) {
        tableMotor = hardwareMap.get(DcMotorEx.class, "linear_motor"); // connect motor

        tableMotor.setDirection(DcMotor.Direction.FORWARD); // set direction
        //tableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        //tableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

        tableMotor.setPower(0);
    }

    public void turn(double power) {
        tableMotor.setPower(power);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Carousel Power", tableMotor.getPower());
    }
}
