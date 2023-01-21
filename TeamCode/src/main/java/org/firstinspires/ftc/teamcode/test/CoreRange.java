package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group = "Test")
@Disabled
public class CoreRange extends OpMode{
    DistanceSensor distanceSensor;
    OpticalDistanceSensor opticalDistanceSensor;
    IrSeekerSensor irSensor;

    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        opticalDistanceSensor = hardwareMap.get(OpticalDistanceSensor.class, "optical");
        irSensor = hardwareMap.get(IrSeekerSensor.class, "irSensor");
    }

    @Override
    public void start() {
        
    }

    @Override
    public void loop() {
        double dist = distanceSensor.getDistance(DistanceUnit.CM);
        double optical = opticalDistanceSensor.getLightDetected();

        telemetry.addData("Distance: ", dist);
        telemetry.addData("Light level", optical);
        telemetry.update();
    }

}
