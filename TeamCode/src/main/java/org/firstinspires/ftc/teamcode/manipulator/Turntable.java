package org.firstinspires.ftc.teamcode.manipulator;

import static org.firstinspires.ftc.teamcode.highlevel.Master.odometryAlg;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.highlevel.Master;

public class Turntable {
    // Make sure that motor starts at 0 ticks
    public static DcMotorEx tableMotor; // Add this to Master
    private static final double TURNTABLE_RATIO = 0.16; // Ratio for angle change
    private static final double TURNTABLE_MOTOR_GEARING =  (double) 1 / 60; // Inherent gearing for turntable motor
    private static final double OVERALL_RATIO = TURNTABLE_RATIO * TURNTABLE_MOTOR_GEARING;

    // Auxillary variables, add these to Master
    public static double turntableAngle; // Radians, as always

    // Internal use variables
    private int neededTicks; // Displacement or target position

    public Turntable(HardwareMap hardwareMap) {
        tableMotor = hardwareMap.get(DcMotorEx.class, "Turntable Motor"); // connect motor

        tableMotor.setDirection(DcMotor.Direction.FORWARD); // set direction
        tableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        tableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run to position?
        tableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

        turntableAngle = 0.0;
        neededTicks = 0;
    }

    public void turnTo(double angle){ // Turntable angle
        turntableAngle = tableMotor.getCurrentPosition() * OVERALL_RATIO;
        neededTicks = (int) ((angle) / OVERALL_RATIO);

        tableMotor.setTargetPosition(neededTicks);
        if(tableMotor.getTargetPosition() < tableMotor.getCurrentPosition()){ // Note that this is geared, directions will look reversed
            tableMotor.setPower(1.0);// Stable enough for max torque?
        }else{
            tableMotor.setPower(-1.0);
        }
    }

    public void turnBy(double angleChange){ // Turntable angle
        turntableAngle = tableMotor.getCurrentPosition() * OVERALL_RATIO;
        neededTicks = (int) (angleChange / OVERALL_RATIO);

        tableMotor.setTargetPosition(neededTicks + tableMotor.getCurrentPosition());
        if(tableMotor.getTargetPosition() < tableMotor.getCurrentPosition()){ // Note that this is geared, directions will look reversed
            tableMotor.setPower(1.0);// Stable enough for max torque?
        }else{
            tableMotor.setPower(-1.0);
        }
    }

    public void reset(){ // Don't tangle the wires lol, run this intermittently
        tableMotor.setTargetPosition(0);
        if(tableMotor.getCurrentPosition() < 0){
            tableMotor.setPower(-1.0); // Directions reversed due to gearing
        }else{
            tableMotor.setPower(1.0);
        }
    }

    @Override
    public String toString(){
        return String.format("Turntable motor position: %d\nTurntable angle from horizontal: %03.2f\nMotor power %2.1f", tableMotor.getCurrentPosition(), turntableAngle, tableMotor.getPower());
    }

    @Deprecated
    public void turn(double power) {
        tableMotor.setPower(power);
    }

    @Deprecated
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Carousel Power", tableMotor.getPower());
    }

}
