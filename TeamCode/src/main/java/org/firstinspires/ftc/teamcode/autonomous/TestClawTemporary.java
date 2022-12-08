package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.manipulator.Claw;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;

@TeleOp(name = "Claw Tester")
public class TestClawTemporary  extends OpMode {
    Servo servo;
//    ServoController controller;


//    LinearSlides slides;

    // Bounds
    double OPEN = 0.425;
    double CLOSED = 0.750;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "claw");
//        controller = servo.getController();
        servo.setPosition(0.0);

        // Testing boundaries
        servo.setPosition(0.0);

        try{
            Thread.sleep(1000);
        }catch(InterruptedException e){

        }

        OPEN = servo.getPosition();

        servo.setPosition(1.0);

        try{
            Thread.sleep(1000);
        }catch(InterruptedException e){

        }
        CLOSED = servo.getPosition();

        telemetry.addData("Open", OPEN);
        telemetry.addData("Closed", CLOSED);

//        slides = new LinearSlides(hardwareMap);
    }

    boolean adjustingOpen = false;

    @Override
    public void loop() {
        telemetry.addData("Servo reference", servo);

        telemetry.addData("OPEN", OPEN);
        telemetry.addData("CLOSED", CLOSED);

        telemetry.update();

        if(gamepad1.dpad_left || gamepad1.dpad_right){
            adjustingOpen = !adjustingOpen;
        }

        if(gamepad1.dpad_up && adjustingOpen){
            OPEN += 0.001;
        }else if(gamepad1.dpad_down && adjustingOpen){
            OPEN -= 0.001;
        }else if(gamepad1.dpad_up && !adjustingOpen){
            CLOSED += 0.001;
        }else if(gamepad1.dpad_down && !adjustingOpen){
            CLOSED -= 0.001;
        }

        if(gamepad1.left_bumper){

            servo.setPosition(OPEN);
//            claw.servo.setDirection(Servo.Direction.FORWARD);


        }else if(gamepad1.right_bumper){
            servo.setPosition(CLOSED);
//            claw.servo.setDirection(Servo.Direction.REVERSE);
        }

//        LinearSlides.extensionMotor.setPower(-gamepad1.right_stick_y);
    }
}
