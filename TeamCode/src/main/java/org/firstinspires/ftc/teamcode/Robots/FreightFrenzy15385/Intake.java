package org.firstinspires.ftc.teamcode.Robots.FreightFrenzy15385;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotor surgicalTubingMotor; //intake
    private Servo intakeHolderServo; //intake holder

    public Intake(HardwareMap hardwareMap){
        surgicalTubingMotor = hardwareMap.get(DcMotor.class, "surgical_tubing");
        intakeHolderServo = hardwareMap.get(Servo.class, "intake_holder");
    }

    public void surgicalTubingOn() {
        surgicalTubingMotor.setPower(-1);
    }

    public void surgicalTubingOff() {
        surgicalTubingMotor.setPower(0);
    }

    public void returnIntakeHolder() {
        intakeHolderServo.setPosition(1);
    }

    public void dropIntake() {
        intakeHolderServo.setPosition(0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Surgical Tubing Motor Power", surgicalTubingMotor.getPower());
        telemetry.addData("Intake Holder Servo Position", intakeHolderServo.getPosition());
    }

    public void testMotor(double power) {
        surgicalTubingMotor.setPower(power);
    }
}
