package org.firstinspires.ftc.teamcode.manipulator;

import static org.firstinspires.ftc.teamcode.manipulator.Turntable.turntableAngle;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.highlevel.Master;
/*
 * (Expansion Hub)
 * Motors:
 * 0    SSM
 * 1    LSEM
 * 2    LSEM2
 */

public class LinearSlides {
    public static DcMotorEx seeSawMotor;
    public static DcMotorEx extensionMotor;
    public static DcMotorEx extensionMotor2;

    public static double [] manipulatorPosition = {0.0, 0.0, 0.0};
    public static double seesawAngle;
    public static double seesawExtensionLength;

    // Internal use variables
    private static double [] displacement;
    private static double angleDisplacement;


    // Manipulator specifications in inches, radians
    public static final double ROOT_HEIGHT = 4.0 + .125 + 2.0; // From ground to linear slide mount
    public static final double STARTING_EXTENDER_LENGTH = 17.0; // Starting length from pivot axle
    // Rotation
    private static final double SEESAW_MOTOR_RATIO = 100; // 60:1 or 40:1 motor?
    public static final double SEESAW_OVERALL_RATIO = Math.PI / (2 * 1400);// 2 * Math.PI * (30.0 / 64.0) / (288 * SEESAW_MOTOR_RATIO); // Angle per tick
    private static final double STARTING_ANGLE = 0.0;// Of the Manipulator, factor in end-effector's center (cone center), in inches
    // Extension
    private static final double EXTENDER_MOTOR_RATIO = 20; // 20:1 or 40:1 motor?
    private static final double EXTENDER_WINCH_RADIUS = 9.4 / 2;
    public static final double EXTENDER_OVERALL_RATIO = 12.0 / 2048; // EXTENDER_WINCH_RADIUS * 2 * Math.PI / (560 * EXTENDER_MOTOR_RATIO); // Inches per tick

    // Temporary stuff
    public static final double [] DEFAULT_INTAKE_DISPLACEMENT = {11.75, -11.75 / 2, -ROOT_HEIGHT};
    public static final double [] DEFAULT_SCORING_DISPLACEMENT = {-11.75, 11.75 / 2, 34.50-ROOT_HEIGHT};

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

    public LinearSlides(HardwareMap hardwareMap){
        seeSawMotor = hardwareMap.get(DcMotorEx.class, "SSM");
        extensionMotor = hardwareMap.get(DcMotorEx.class, "LSEM");
        extensionMotor2 = hardwareMap.get(DcMotorEx.class, "LSEM2");

        seeSawMotor.setDirection(DcMotor.Direction.FORWARD); // set direction, this was made for 1 gear transfer from drive to axle
        seeSawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        seeSawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run to posi  tion?
        seeSawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

        // Extension Motor specifics need to be edited ig
        extensionMotor.setDirection(DcMotor.Direction.REVERSE); // set direction, probably need to change
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run to position?
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

        extensionMotor2.setDirection(DcMotor.Direction.FORWARD); // set direction, probably need to change
        extensionMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        extensionMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run to position?
        extensionMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

//        seeSawMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.0, 0.1, 10.0, 0));
//        extensionMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.0, 0.1, 10.0, 0));
//        extensionMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.0, 0.1, 10.0, 0));

        // Motor kinematics ;)
        // Change this
        manipulatorPosition[0] = STARTING_EXTENDER_LENGTH * Math.cos(turntableAngle) * Math.cos(STARTING_ANGLE);
        manipulatorPosition[1] = STARTING_EXTENDER_LENGTH * Math.sin(turntableAngle) * Math.cos(STARTING_ANGLE);
        manipulatorPosition[2] =  ROOT_HEIGHT + (STARTING_EXTENDER_LENGTH * Math.sin(STARTING_ANGLE));

        displacement = new double[2];
        angleDisplacement = 0.0;
    }

    public double[] getClawCoordinates() {
        double twoDimensionalRadius = seesawExtensionLength * Math.cos(seesawAngle);
        double x = twoDimensionalRadius * Math.cos(turntableAngle);
        double y = twoDimensionalRadius * Math.sin(turntableAngle);
        double height = seesawExtensionLength * Math.sin(seesawAngle);

        return new double[] { x, y, height };
    }

    private static final double MANIPULATOR_BACKSET_DISTANCE = 3.5;
    public void goPointAt(double [] xyzDisplacement){

//        // Make sure to input 3-array for targeted scoring position!!!!!!!! Will have to get angle of robot once it gets to junction, then correct a second time. This is not a one-time algorithm!!!
//        // Actually does everything at the same time, will need to edit based on extension speed (want to minimize extended time for reliability purposes)
//        displacement = xyzDisplacement; // Screw it I don't wanna run the method over and over haha
//        //  Note: displacement from pivot point of linear slide
//        // Move turntable, note that this will turn the turntable 180˚ back if target is behind the pivot, will need to account for ability to swing beyond 90˚ vertical (behind) later
//        if(displacement[0] <= 0){ // Getting displacement angle
//            angleDisplacement = Math.PI + Math.atan(displacement[1] / displacement[0]); // Might wanna use taylor series to approximate atan later since calculation times are gonna be annoying
//        }else{
//            angleDisplacement = Math.atan(displacement[1] / displacement[0]);
//        }


//        // Make sure to input 3-array for targeted scoring position!!!!!!!! Will have to get angle of robot once it gets to junction, then correct a second time. This is not a one-time algorithm!!!
//        // Actually does everything at the same time, will need to edit based on extension speed (want to minimize extended time for reliability purposes)
//        displacement = xyzDisplacement; // Screw it I don't wanna run the method over and over haha
//        //  Note: displacement from pivot point of linear slide
//        // Move turntable, note that this will turn the turntable 180˚ back if target is behind the pivot, will need to account for ability to swing beyond 90˚ vertical (behind) later
//        if(displacement[0] <= 0){ // Getting displacement angle
//            angleDisplacement = Math.PI + Math.atan(displacement[1] / displacement[0]); // Might wanna use taylor series to approximate atan later since calculation times are gonna be annoying
//        }else{
//            angleDisplacement = Math.atan(displacement[1] / displacement[0]);
//        }
//


//        if(!(mecanumDrive == null)){ // For autonomous
//            while(!(Math.abs(angleDisplacement - Autonomous.odometry.getRotationRadians()) < 0.1)){
//                if(angleDisplacement >= Autonomous.odometry.getRotationRadians()){

////                    mecanumDrive.fieldOrientatedDrive(0.0, 0.0, 0.8, mecanumDrive.odometry.getRotationRadians());
//                }else{
////                    mecanumDrive.fieldOrientatedDrive(0.0, 0.0, -0.8);

//                    mecanumDrive.fieldOrientatedDrive(0.0, 0.0, 0.8, mecanumDrive.odometry.getRotationRadians());
//                }else{
//                    mecanumDrive.fieldOrientatedDrive(0.0, 0.0, -0.8);

//                }
//
//                Autonomous.odometry.updatePosition();
//            }
//
//        }else{
//            while(!(Math.abs(angleDisplacement - MainTeleOp.odometry.getRotationRadians()) < 0.1)){
//                if(angleDisplacement >= MainTeleOp.odometry.getRotationRadians()){

////                    MainTeleOp.mecanum.fieldOrientatedDrive(0.0, 0.0, 0.8);
//                }else{
////                    MainTeleOp.mecanum.fieldOrientatedDrive(0.0, 0.0, -0.8);

//                    MainTeleOp.mecanum.fieldOrientatedDrive(0.0, 0.0, 0.8);
//                }else{
//                    MainTeleOp.mecanum.fieldOrientatedDrive(0.0, 0.0, -0.8);

//                }
//
//                MainTeleOp.odometry.updatePosition();
//            }
//        }


        // Pivot seesaw up / down to desired position. Due to the behavior described in the previous step, this step (for now) doesn't have to account for swinging beyond 90˚ vertical
        /*
          /|
         / |
        /\_| <- Angle
         */


//
//        // Pivot seesaw up / down to desired position. Due to the behavior described in the previous step, this step (for now) doesn't have to account for swinging beyond 90˚ vertical
//        /*
//          /|
//         / |
//        /\_| <- Angle
//         */
//

//        if(mecanumDrive == null){ // Case teleOp
//            if(Math.abs(angleDisplacement - MainTeleOp.odometry.getRotationRadians()) > Math.PI){ // Case backwards scoring
//                this.pivotTo(Math.PI + Math.atan(displacement[3] /
//                        Math.sqrt(MANIPULATOR_BACKSET_DISTANCE*MANIPULATOR_BACKSET_DISTANCE + displacement[0]*displacement[0] + displacement[1]*displacement[1])));
//            }else{
//                this.pivotTo(Math.atan(displacement[3] /
//                        Math.sqrt(MANIPULATOR_BACKSET_DISTANCE*MANIPULATOR_BACKSET_DISTANCE + displacement[0]*displacement[0] + displacement[1]*displacement[1])));
//            }
//        }else{
//            if(Math.abs(angleDisplacement - Autonomous.odometry.getRotationRadians()) > Math.PI){ // Case backwards scoring
//                this.pivotTo(Math.PI + Math.atan(displacement[3] /
//                        Math.sqrt(MANIPULATOR_BACKSET_DISTANCE*MANIPULATOR_BACKSET_DISTANCE + displacement[0]*displacement[0] + displacement[1]*displacement[1])));
//            }else{
//                this.pivotTo(Math.atan(displacement[3] /
//                        Math.sqrt(MANIPULATOR_BACKSET_DISTANCE*MANIPULATOR_BACKSET_DISTANCE + displacement[0]*displacement[0] + displacement[1]*displacement[1])));
//            }
//        }

        // Extend
//        this.extendTo(Math.sqrt(MANIPULATOR_BACKSET_DISTANCE*MANIPULATOR_BACKSET_DISTANCE + // Added to accomodate backset manipulator
//                displacement[0]*displacement[0] + displacement[1]*displacement[1] + displacement[2]*displacement[2]));
    }

    public void extendTo(double inches){ // Inches
        extensionMotor.setTargetPosition((int) ((inches - STARTING_EXTENDER_LENGTH) / EXTENDER_OVERALL_RATIO));
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor2.setTargetPosition(extensionMotor.getCurrentPosition());
        extensionMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(extensionMotor.getTargetPosition() > extensionMotor.getCurrentPosition()){
            extensionMotor.setPower(.5);
            extensionMotor2.setPower(-.5);
        }else{
            extensionMotor.setPower(-.5);
            extensionMotor2.setPower(.5);
        }
    }

    public void extendBy(double power){
//        if(extensionMotor.getCurrentPosition() <= 0) {
//            extensionMotor.setVelocity(power * 100, AngleUnit.RADIANS);
//            extensionMotor2.setVelocity(- power * 100, AngleUnit.RADIANS);
//            if(extensionMotor.getCurrentPosition() > -50 && extensionMotor2.getCurrentPosition() > -50){
                extensionMotor.setPower(power);

                extensionMotor2.setTargetPosition(extensionMotor.getCurrentPosition());
                extensionMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(extensionMotor2.getTargetPosition() > extensionMotor2.getCurrentPosition()){
                    extensionMotor2.setPower(0.5);
                }else{
                    extensionMotor2.setPower(-0.5);
                }
//            }else{
//                extensionMotor.setPower(0);
//                extensionMotor2.setPower(0);

//            }
//                while(extensionMotor.getCurrentPosition() < 0.0 || extensionMotor2.getCurrentPosition() < 0.0){
//                    extensionMotor.setPower(0.5);
//                    extensionMotor2.setPower(0.5);
//                }

//        } else {
//            extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
    }

    public void pivotTo(double angleRadians) { // Radians, zero is horizontal
        seesawAngle = seeSawMotor.getCurrentPosition() * SEESAW_OVERALL_RATIO;
//        if(seesawAngle < Math.PI / 2 && seesawAngle > 0.0) {
            seeSawMotor.setTargetPosition((int) (angleRadians / SEESAW_OVERALL_RATIO));
            seeSawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(seeSawMotor.getTargetPosition() > seeSawMotor.getCurrentPosition()){
                seeSawMotor.setPower(.5);
            }else{
                seeSawMotor.setPower(-.5);
            }
//        }
    }

    public void pivotBy(double power) {

        seeSawMotor.setPower(power);


//        seeSawMotor.setTargetPosition((int)(power * 100 + seeSawMotor.getCurrentPosition()));
//        seeSawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        seeSawMotor.setPower(1);

    }

    public void update(){ // Run this as much as applicable
        seesawExtensionLength = ((.5 * extensionMotor.getCurrentPosition() + .5 * extensionMotor2.getCurrentPosition()) * EXTENDER_OVERALL_RATIO) + STARTING_EXTENDER_LENGTH;
//        if(seesawAngle > -1.0){
            seesawAngle = (seeSawMotor.getCurrentPosition() * SEESAW_OVERALL_RATIO); // Assuming starting angle is 0
//        }

//        manipulatorPosition[0] = seesawExtensionLength * Math.cos(Master.turntableAngle) * Math.cos(seesawAngle);
//        manipulatorPosition[1] = seesawExtensionLength * Math.sin(Master.turntableAngle) * Math.cos(seesawAngle);
//        manipulatorPosition[2] = seesawExtensionLength * Math.sin(seesawAngle);
    }


    // Temporary, first tournament
    public void grabFromDefaultScoringPosition(){
//        claw.openClaw();
//        claw.waitForOpenClaw();
//        this.goPointAt(DEFAULT_INTAKE_DISPLACEMENT);
//        claw.closeClaw();
    }

    public void scoreFromDefaultScoringPosition() {
//        if(Vector.lengthOf(Vector.add(Vector.neg(getClawCoordinates()), DEFAULT_SCORING_DISPLACEMENT)) > 0.1){
//            this.goPointAt(DEFAULT_SCORING_DISPLACEMENT);
//        }
//
//        claw.openClaw();
    }

    public void resetEncoders() {
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        seeSawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("\nLINEAR SLIDES + SEESAW");
        telemetry.addLine(String.format("Seesaw Motor Power: %f", seeSawMotor.getPower()));
        telemetry.addLine(String.format("Seesaw Motor Velocity: %f", seeSawMotor.getVelocity()));
        telemetry.addLine(String.format("Seesaw Current Position: %d", seeSawMotor.getCurrentPosition()));
        telemetry.addLine(String.format("Seesaw Target Position: %d", seeSawMotor.getTargetPosition()));
        telemetry.addLine(String.format("Extension Motors Powers: %f %f", extensionMotor.getPower(), extensionMotor2.getPower()));
        telemetry.addLine(String.format("Extension Motors Velocities: %f %f", extensionMotor.getVelocity(), extensionMotor2.getVelocity()));
        telemetry.addLine(String.format("Extensions Current Positions: %d %d", extensionMotor.getCurrentPosition(), extensionMotor2.getCurrentPosition()));
        telemetry.addLine(String.format("Extensions Target Positions: %d %d", extensionMotor.getTargetPosition(), extensionMotor2.getTargetPosition()));

//        telemetry.addData("\nController target angle",);
    }
}
