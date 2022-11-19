package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.manipulator.Claw;

@TeleOp(name = "Claw Tester")
public class TestClawTemporary  extends OpMode {
    Claw claw;

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper){

            claw.openClaw();
        }else if(gamepad1.right_bumper){
            claw.closeClaw();
        }
    }
}
