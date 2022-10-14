package org.firstinspires.ftc.teamcode.Robots.Demo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DemoIntake {
    private DcMotor intakeMotor; //intake

    public DemoIntake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
    }

    public void surgicalTubingOn() {
        intakeMotor.setPower(-1);
    }

    public void surgicalTubingOff() {
        intakeMotor.setPower(0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Surgical Tubing Motor Power", intakeMotor.getPower());
    }
}
