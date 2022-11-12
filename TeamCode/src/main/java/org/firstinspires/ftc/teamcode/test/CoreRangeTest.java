package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class CoreRangeTest extends OpMode {
    ModernRoboticsI2cRangeSensor modernRoboticsI2cRangeSensor;

    @Override
    public void init() {
        modernRoboticsI2cRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "corerange");
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addLine("disance" + modernRoboticsI2cRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

}
