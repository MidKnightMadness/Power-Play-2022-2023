package org.firstinspires.ftc.teamcode.robots.ultimategoal7854;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Chassis {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    public Chassis(HardwareMap hardwareMap){
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");

        // Set Directions
        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);

        // Set Motor Mode
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set Zero Power Behavior
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stops Motors on INIT
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void drive(double x, double y, double rotate) {
        fr.setPower(-x - y - rotate);
        fl.setPower(-x + y - rotate);
        br.setPower(-x + y + rotate);
        bl.setPower(-x - y + rotate);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("FR Motor Position", fr.getCurrentPosition());
        telemetry.addData("FL Motor Position", fl.getCurrentPosition());
        telemetry.addData("BR Motor Position", br.getCurrentPosition());
        telemetry.addData("BL Motor Position", bl.getCurrentPosition());
    }

}
