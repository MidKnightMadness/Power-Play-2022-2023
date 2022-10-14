package org.firstinspires.ftc.teamcode.robots.freightfrenzy7854;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**CONTROLS(NOTE: Will change based on Driver's preference)**
 *
 * =======================================================
 *
 * Gamepad 1
 * Joysticks = driving
 * Right Bumper = Slow Mode
 * A button = Resets the outtake flap(Blocks the game elements)
 * B button = Opens the outtake flap half-way full
 *
 *=========================================================
 *
 * Gamepad 2
 * Left trigger = Intake(in)
 * Right Trigger = Intake(out/reverse)
 * A button = Carousel on/off(toggle)
 * B button = Direction of Carousel(CW/CCW)
 * X Button = Closes the arm(Grabs the team-element)
 * Y Button = Opens the arm(Releases the team-element)
 * DPad LEFT = Move teamElement arm to the Left(Inside the robot/Default)
 * DPad RIGHT =  Move teamElement arm to the Right(Outside of the robot)
 * DPad UP = Winch UP
 * DPad DOWN = Winch DOWN
 *
 *===========================================================
 *
 */

@TeleOp
public class FreightFrenzy7854 extends LinearOpMode{

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DcMotor carouselTurningMotor;
    DcMotor winch;

    public void runOpMode(){
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        carouselTurningMotor = hardwareMap.dcMotor.get("carouselTurningMotor");
        winch = hardwareMap.get(DcMotor.class, "winch");

        //fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        float[] power = new float[4];
        
        waitForStart();

        //Carousel car = new Carousel(hardwareMap);
        Chassis ch = new Chassis(hardwareMap);
        Intake it = new Intake(hardwareMap);

        while (opModeIsActive()) {

            //Intake
            it.intakeHold(telemetry, this.gamepad2.left_trigger, this);

            //it.flapFullOpen(this.gamepad1.x);
            it.flapHalfOpen(this.gamepad1.b);
            it.reverse(telemetry, this.gamepad2.right_trigger, this);
            it.resetFlap(this.gamepad1.a);
            telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", gamepad1.right_trigger);
            
            //drive
            //ch.slowMode(this.gamepad1.right_bumper);
            power = ch.mecanumDr(this.gamepad1.left_stick_x,this.gamepad1.left_stick_y,this.gamepad1.right_stick_x,this.gamepad1.right_stick_y);

            //assigning speeds
            fl.setPower(power[0]);
            fr.setPower(-power[1]);
            bl.setPower(power[2]);
            br.setPower(-power[3]);

            /*
            fl.setPower(-.5); //(+) for 2020-21 Mayhem bot | (-) for 2021-22
            fr.setPower(.5); //(-) for 2020-21 Mayhem bot | (+) for 2021-22
            bl.setPower(.5); //(+) for 2020-21 Mayhem bot | (-) for 2021-22
            br.setPower(-.5);*/
            /* --------------------------------------

            Telemetry Stuff

            -------------------------------------- */



            telemetry.addData("Front Left Motor",power[0]);
            telemetry.addData("Front Right Motor",power[1]);
            telemetry.addData("Back Left Motor",power[2]);
            telemetry.addData("Back Right Motor",power[3]);

            telemetry.update();
        }
    }
}