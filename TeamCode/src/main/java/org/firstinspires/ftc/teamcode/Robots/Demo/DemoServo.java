package org.firstinspires.ftc.teamcode.Robots.Demo;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DemoServo {
    CRServo servo;

    public DemoServo(HardwareMap hardwareMap){
        servo = hardwareMap.get(CRServo.class, "servo");
    }

    // the range for power is -1 to 1
    public void spinBlue() {
        servo.setPower(1.0);
    }

    public void spinRed() {
        servo.setPower(-1.0);
    }

    public void spinOff() {
        servo.setPower(0.0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Carousel Power", servo.getPower());
    }
}
