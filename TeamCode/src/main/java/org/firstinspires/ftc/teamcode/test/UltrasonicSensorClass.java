package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

@TeleOp
public class UltrasonicSensorClass extends OpMode {

    UltrasonicSensor ultrasonicSensor;

    @Override
    public void init() {
        ultrasonicSensor = hardwareMap.get(UltrasonicSensor.class, "ultrasonic");
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addLine("" + ultrasonicSensor.getUltrasonicLevel());
        telemetry.update();
    }
}

