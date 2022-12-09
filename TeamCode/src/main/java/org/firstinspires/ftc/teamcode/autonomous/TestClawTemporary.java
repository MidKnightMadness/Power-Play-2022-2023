package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.manipulator.Claw;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;

@TeleOp(name = "Claw Tester")
public class TestClawTemporary  extends OpMode {


    Claw claw;

    LinearSlides slides;

    // Bounds
    double OPEN = 0.425;
    double CLOSED = 0.750;

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
        slides = new LinearSlides(hardwareMap);

        claw.pivotTo(-Math.PI / 4);
    }

    boolean adjustingOpen = false;

    @Override
    public void loop() {
        if(gamepad1.left_bumper || gamepad1.right_bumper){
            if(claw.open){
                claw.closeClaw();
            }else{
                claw.openClaw();
            }
        }

        claw.pivotTo(- (LinearSlides.seeSawMotor.getCurrentPosition() * LinearSlides.SEESAW_OVERALL_RATIO) -  (Math.PI / 4));

        LinearSlides.extensionMotor.setPower(-gamepad1.right_stick_y);
        LinearSlides.extensionMotor2.setPower(-gamepad1.right_stick_y);
        LinearSlides.seeSawMotor.setPower(-gamepad1.left_stick_y);


//        slides.extendBy(-gamepad1.right_stick_y);
    }
}
