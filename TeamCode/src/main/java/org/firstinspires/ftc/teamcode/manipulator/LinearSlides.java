package org.firstinspires.ftc.teamcode.manipulator;

import static org.firstinspires.ftc.teamcode.archive.Turntable.turntableAngle;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public double seesawAngle;
    public double seesawExtensionLength;

    // Internal use variables
    private static double [] displacement;
    private static double angleDisplacement;

    // Manipulator specifications in inches, radians
    public static final double ROOT_HEIGHT = 4.0 + .125 + 2.0; // From ground to linear slide mount
    public static final double STARTING_EXTENDER_LENGTH = 19.0; // Starting length from pivot axle
    // Rotation
    private static final double SEESAW_MOTOR_RATIO = 100; // 60:1 or 40:1 motor?
    public static final double SEESAW_OVERALL_RATIO = Math.PI / (2 * 1417); // Angle per tick
    private static double STARTING_ANGLE = 0.0;// Temporary, for testing MainTeleOp w/ manipulator starting down
    // Extension
    private static final double EXTENDER_MOTOR_RATIO = 20; // 20:1 or 40:1 motor?
    private static final double EXTENDER_WINCH_RADIUS = 9.4 / 2;
    public static final double EXTENDER_OVERALL_RATIO = 16.5 / 3132; // EXTENDER_WINCH_RADIUS * 2 * Math.PI / (560 * EXTENDER_MOTOR_RATIO); // Inches per tick

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

    public LinearSlides(HardwareMap hardwareMap, double startingAngle){
        seeSawMotor = hardwareMap.get(DcMotorEx.class, "SSM");
        extensionMotor = hardwareMap.get(DcMotorEx.class, "LSEM");
        extensionMotor2 = hardwareMap.get(DcMotorEx.class, "LSEM2");

        seeSawMotor.setDirection(DcMotor.Direction.FORWARD); // set direction, this was made for 1 gear transfer from drive to axle
        seeSawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        seeSawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run to posi  tion?
        seeSawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

        // Extension Motor specifics need to be edited ig
        extensionMotor.setDirection(DcMotor.Direction.FORWARD); // set direction, probably need to change
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run to position?
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

        extensionMotor2.setDirection(DcMotor.Direction.REVERSE); // set direction, probably need to change
        extensionMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set motor mode
        extensionMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run to position?
        extensionMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set zero power behavior

//        seeSawMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.0, 0.1, 10.0, 0));
//        extensionMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.0, 0.1, 10.0, 0));
//        extensionMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.0, 0.1, 10.0, 0));

        // Motor kinematics ;)
        // Change this
        STARTING_ANGLE = startingAngle;
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

    public static final double MANIPULATOR_BACKSET_DISTANCE = 3.5;

    public void extendTo(double inches){ // Inches

        double distance1 = (inches - STARTING_EXTENDER_LENGTH) / EXTENDER_OVERALL_RATIO-extensionMotor.getCurrentPosition();
        double power = distance1/Math.max(100, Math.abs(distance1));
        extensionMotor.setPower(0.6 * power);
        extensionMotor2.setPower(0.6 * power);
    }

    public void extendBy(double power){
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double truePower = 0.0;

        if (seesawExtensionLength - STARTING_EXTENDER_LENGTH <= 0){
            truePower = Math.max(power, 0.1);
        } else if (seesawExtensionLength >= 34.0){
            truePower = Math.min(power, 0.1);
        } else {
            truePower = power;
        }

        extensionMotor.setPower(0.6 * truePower);
        extensionMotor2.setPower(0.6 * truePower);

    }

    private int ticksDifference = 0;

    public void pivotTo(double targetAngle) { // Radians, zero is horizontal

        // Needs to not fling the robot over / flip it by going too fast, so implemented linear decrease in power as approaching target

        brake = 0.0; //0.00005 * (seesawExtensionLength / 2) * Math.cos(seesawAngle);
        ticksDifference = (int) ((targetAngle - seesawAngle) / SEESAW_OVERALL_RATIO); // Minimal play involved now

        seeSawMotor.setPower(ticksDifference * 0.4 / Math.max(Math.abs(ticksDifference), 1000) + brake);

    }

    double brake = 0.0;
    double seesawPowerProfile = 0.0;
    public void pivotBy(double power) {
        brake = 0.0; // 0.00005 * (seesawExtensionLength / 2) * Math.cos(seesawAngle);
        seesawPowerProfile = (1 / (1 + Math.exp(seesawAngle * seesawAngle * seesawAngle + 70.0 * Math.PI / 180.0)))
                * (seesawAngle * seesawAngle * seesawAngle + 80 * Math.PI / 180)
                / 0.27027;

        if(Math.abs(power) < 0.1){
            seeSawMotor.setPower(brake);

        }else{
//
            // Power profiling:
            //    |____  | <- upper limit normalized
            //    |    \_| <- 2π/3
            // Should go to 0 for moving up when close to 120˚, multiplying just maxes out power
            // Solves issue of impulse in short period of time causing flips

            // 2-6 Update - implemented logistic curve to solve issue, needs less power earlier due to lag in motor update rate

            if(seesawAngle >= 107 * Math.PI / 180){ // Above 90˚, max backwards (positive) power restricted below profile, but forwards (negative) power not
                seeSawMotor.setPower(Math.min(brake, 0.5 * power));
            }else if(seesawAngle <= 0.0){ // Won't let manipulator press into top plate
                seeSawMotor.setPower(Math.max(0.5 * power, brake));
            }else{
                seeSawMotor.setPower(power * .5);
            }

        }
    }

    private double previousAngle = 0.00;

    public void update(){ // Run this as much as applicable
        previousAngle = seesawAngle;

        seesawExtensionLength = (extensionMotor.getCurrentPosition() * EXTENDER_OVERALL_RATIO) + STARTING_EXTENDER_LENGTH;
        seesawAngle = (seeSawMotor.getCurrentPosition() * SEESAW_OVERALL_RATIO) + STARTING_ANGLE;
    }

    public void resetEncoders() {
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        seeSawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addLine("LINEAR SLIDES + SEESAW");
        telemetry.addLine(String.format("Seesaw Motor Power: %f", seeSawMotor.getPower()));
        telemetry.addLine(String.format("Seesaw Motor Velocity: %f", seeSawMotor.getVelocity()));
        telemetry.addLine(String.format("Seesaw Current Position: %d", seeSawMotor.getCurrentPosition()));
        telemetry.addLine(String.format("Seesaw Target Position: %d", seeSawMotor.getTargetPosition()));
        telemetry.addLine(String.format("Extension Motors Powers: %f %f", extensionMotor.getPower(), extensionMotor2.getPower()));
        telemetry.addLine(String.format("Extension Motors Velocities: %f %f", extensionMotor.getVelocity(), extensionMotor2.getVelocity()));
        telemetry.addLine(String.format("Extensions Current Positions: %d %d", extensionMotor.getCurrentPosition(), extensionMotor2.getCurrentPosition()));
        telemetry.addLine(String.format("Extensions Target Positions: %d %d", extensionMotor.getTargetPosition(), extensionMotor2.getTargetPosition()));

        telemetry.addData("\nExtension Length", seesawExtensionLength);
        telemetry.addData("Seesaw angle", seesawAngle * 180 / Math.PI);
//        telemetry.addData("Relative calculated extender length, extension motor 1", (extensionMotor.getCurrentPosition() * EXTENDER_OVERALL_RATIO));
//        telemetry.addData("Calculated extender length, extension motor 2", (extensionMotor2.getCurrentPosition() * EXTENDER_OVERALL_RATIO) + STARTING_EXTENDER_LENGTH);
//        telemetry.addData("Calculated extender length, averaged", ((0.5 * extensionMotor2.getCurrentPosition() + 0. * extensionMotor.getCurrentPosition()) * EXTENDER_OVERALL_RATIO) + STARTING_EXTENDER_LENGTH);
    }
}
