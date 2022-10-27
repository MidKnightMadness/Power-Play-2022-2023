package org.firstinspires.ftc.teamcode.manipulator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DoubleReverse4Bar {
    private DcMotorEx liftMotor;
    private int motorStartPosition;
    private static final double OVERALL_RATIO = 1.0; // ticks to inches raised, not negative
    private static final double STARTING_HEIGHT = 8.0; // inches, should be default
    public static double manipulatorHeight = STARTING_HEIGHT;

    public DoubleReverse4Bar(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift_motor"); // connect motor
        motorStartPosition = liftMotor.getCurrentPosition();

        liftMotor.setDirection(DcMotor.Direction.FORWARD); // set direction
        //linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        //linearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(0);
    }

    public void raiseTo(double height) { // Does conversion to ticks in the method
        liftMotor.setTargetPosition((int) ((height - STARTING_HEIGHT) / OVERALL_RATIO)); // Assuming that starting ticks is 0
        if(liftMotor.getTargetPosition() > liftMotor.getCurrentPosition()){
            liftMotor.setPower(1.0); // Assuming 1 gearing, pulling the thign down
        }else{
            liftMotor.setPower(-1.0);
        }
        liftMotor.setPower(1.0);
        // Needs to use turntable method to achieve correct angle, robot needs to drive to correct radius from junction
    }

    public void update(){ // Run this if other methods are dependant on the data here
        manipulatorHeight = STARTING_HEIGHT + (liftMotor.getCurrentPosition() * OVERALL_RATIO);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Motor Current", liftMotor.getCurrentPosition());
        telemetry.addData("Lift Motor Target", liftMotor.getTargetPosition());
    }
}
