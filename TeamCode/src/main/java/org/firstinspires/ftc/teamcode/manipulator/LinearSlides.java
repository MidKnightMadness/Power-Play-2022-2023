package org.firstinspires.ftc.teamcode.manipulator;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain.*;
import java.math.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.highlevel.Master;

import static org.firstinspires.ftc.teamcode.highlevel.Master.invSqrt;
import static org.firstinspires.ftc.teamcode.highlevel.Master.turntable;
import static org.firstinspires.ftc.teamcode.manipulator.Turntable.turntableAngle; // Remove this in final version

// This comment was pushed today
public class LinearSlides {
    // Add all of these to Master
    public static DcMotorEx seeSawMotor;
    public static DcMotorEx extensionMotor;
    public static Vector manipulatorPosition;
    private static HardwareMap hardwareMap;
    public static double seesawAngle;
    public static double seesawExtensionLength;

    // Internal use variables
    private static double [] displacement;
    private static double angleDisplacement;
    private static double ticksDisplacement;

    // Manipulator specifications in inches, radians
    public static final double ROOT_HEIGHT = 8.0; // From ground to linear slide mount
    private static final double STARTING_EXTENDER_LENGTH = 9.0; // Starting length from pivot axle
    // Rotation
    private static final double SEESAW_MOTOR_RATIO = 60; // 60:1 or 40:1 motor?
    private static final double SEESAW_OVERALL_RATIO = 2 * Math.PI * (30 / 64) / (4096 * SEESAW_MOTOR_RATIO); // Angle per tick
    private static final double STARTING_ANGLE = 0.0;// Of the Manipulator, factor in end-effector's center (cone center), in inches
    // Extension
    private static final double EXTENDER_MOTOR_RATIO = 20; // 20:1 or 40:1 motor?
    private static final double PULLEY_RADIUS = 1.0; // Radius of pulley interacting with string
    private static final double EXTENDER_OVERALL_RATIO = 2 * Math.PI / (4096 * EXTENDER_MOTOR_RATIO); // Inches per tick

    /* Manipulator diagram:

            ==/\=============== <- Length from center of pivot w/ extension
              ||                <- Root
          ____||____            <- Turntable


     Extension Mechanism:
           _     ________ <- Secondary section
        ==/o\========     <- Primary section

     1. String from extension motor to end of manipulator, longer when retracted
     2. Surgical tubing between end and pivot of manipulator to bias manipulator towards retracted
     3. Pull string to extend, roll back to retract
     */

    public LinearSlides(){
        seeSawMotor = hardwareMap.get(DcMotorEx.class, "Seesaw Motor");
        extensionMotor = hardwareMap.get(DcMotorEx.class, "Linear Slide Extension Motor");

        seeSawMotor.setDirection(DcMotor.Direction.FORWARD); // set direction, this was made for 1 gear transfer from drive to axle
        seeSawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        seeSawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run to position?
        seeSawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

        // Extension Motor specifics need to be edited ig
        extensionMotor.setDirection(DcMotor.Direction.FORWARD); // set direction, probably need to change
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run to position?
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

        // Motor kinematics ;)
        // Note: initialize turntable before manipulator!!!
        manipulatorPosition = new Vector(new double [] {0.0, 0.0, 0.0}); // Idk if this works lol

        manipulatorPosition.set(0, (STARTING_EXTENDER_LENGTH) * Math.cos(turntableAngle) * Math.cos(STARTING_ANGLE));
        manipulatorPosition.set(1, (STARTING_EXTENDER_LENGTH) * Math.sin(turntableAngle) * Math.cos(STARTING_ANGLE));
        manipulatorPosition.set(2, ROOT_HEIGHT + (STARTING_EXTENDER_LENGTH * Math.sin(STARTING_ANGLE)));

        displacement = new double[2];
        angleDisplacement = 0.0;
        ticksDisplacement = 0.0;
    }

    public void goPointAt(double [] xyzDisplacement){ // Make sure to input 3-array for targeted scoring position!!!!!!!! Will have to get angle of robot once it gets to junction, then correct a second time. This is not a one-time algorithm!!!
        // Actually does everything at the same time, will need to edit based on extension speed (want to minimize extended time for reliability purposes)
        displacement = xyzDisplacement; // Screw it I don't wanna run the method over and over haha
//  Note: displacement from pivot point of linear slide
        // Move turntable, note that this will turn the turntable 180˚ back if target is behind the pivot, will need to account for ability to swing beyond 90˚ vertical (behind) later
        if(displacement[0] <= 0){ // Getting displacement angle
            angleDisplacement = Math.PI - Math.atan(displacement[1] / displacement[0]); // Might wanna use taylor series to approximate atan later since calculation times are gonna be annoying
        }else{
            angleDisplacement = Math.atan(displacement[1] / displacement[0]);
        }
        turntable.turnTo(angleDisplacement);

        // Pivot seesaw up / down to desired position. Due to the behavior described in the previous step, this step (for now) doesn't have to account for swinging beyond 90˚ vertical
        /*
          /|
         / |
        /\_| <- Angle
         */
        this.pivotTo(Math.atan(displacement[3] / invSqrt(displacement[0]*displacement[0] + displacement[1]*displacement[1])));

        // Extend
        this.extendTo(Math.sqrt(displacement[0]*displacement[0] + displacement[1]*displacement[1] + displacement[2]*displacement[2]));
    }

    public void extendTo(double target){ // Inches
        extensionMotor.setTargetPosition((int) ((target - STARTING_EXTENDER_LENGTH) / EXTENDER_OVERALL_RATIO));
        if(extensionMotor.getTargetPosition() < extensionMotor.getCurrentPosition()){
            extensionMotor.setPower(-1.0);
        }else{
            extensionMotor.setPower(1.0);
        }
    }

    public void pivotTo(double targetAngle){ // Radians
        seeSawMotor.setTargetPosition((int) (targetAngle / SEESAW_OVERALL_RATIO));
        if(seeSawMotor.getTargetPosition() < seeSawMotor.getCurrentPosition()){ // Remember, motor is geared so direction reversed
            seeSawMotor.setPower(-1.0); // To go down, set power to negative, might have to reverse this based on motor packaging
        }else{
            seeSawMotor.setPower(1.0);
        }
    }

    public void update(){ // Run this as much as applicable
        seesawExtensionLength = (seeSawMotor.getCurrentPosition() * EXTENDER_OVERALL_RATIO) - STARTING_EXTENDER_LENGTH;
        seesawAngle = (seeSawMotor.getCurrentPosition() * SEESAW_OVERALL_RATIO); // Assuming starting angle is 0
    }
}
