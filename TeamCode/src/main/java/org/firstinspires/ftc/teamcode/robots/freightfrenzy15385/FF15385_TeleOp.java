package org.firstinspires.ftc.teamcode.robots.freightfrenzy15385;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
/*
Player 1 (gamepad1)
left stick      (hold)      shifting
right stick     (hold)      rotating
dpad up         (hold)      go to shipping hub position

Player 2 (gamepad2)
y               (toggle)    catapult to upper
b               (toggle)    catapult to middle
a               (toggle)    catapult to lower
left trigger    (toggle)    catapult flap
left bumper     (toggle)    team shipping element lift motor
left trigger    (toggle)    team shipping element lift servo
left bumper     (toggle)    surgical tubing
dpad down       (toggle)    rotate catapult head left
dpad left       (hold)      spin carousel left
dpad right      (hold)      spin carousel right
*/

@TeleOp(name="Freight Frenzy - 15385 Mayhem", group="Practice Robots")
@Disabled
public class FF15385_TeleOp extends OpMode {
    SampleDrive drive;
    Catapult catapult;
    Carousel carousel;
    Intake intake;
    Lift lift;

    private DistanceSensor sensorDistanceL; //left front sensor
    private ModernRoboticsI2cRangeSensor sensorRangeM; //middle front range sensor
    private DistanceSensor sensorDistanceR; //right front sensor

    private boolean lastPressedCatapultUpper = false;
    private boolean lastPressedCatapultMiddle = false;
    private boolean lastPressedCatapultLower = false;
    private boolean catapultUpperToggle = false;
    private boolean catapultMiddleToggle = false;
    private boolean catapultLowerToggle = false;
    private boolean lastPressedFlap = false;
    private boolean flapToggle = false;
    private boolean lastPressedCatapultHeadLeft = false;
    private boolean catapultHeadLeftToggle = false;

    private boolean lastPressedSurgical = false;
    private boolean surgicalToggle = false;
    private boolean lastPressedIntakeHolder = false;
    private boolean intakeHolderToggle = false;
    private boolean lastPressedLiftMotor = false;
    private boolean liftMotorToggle = false;
    private boolean lastPressedLiftServo = false;
    private boolean liftServoToggle = false;

    @Override
    public void init() {
        drive = new SampleDrive(hardwareMap);
        catapult = new Catapult(hardwareMap);
//        carousel = new Carousel(hardwareMap);
//        intake = new Intake(hardwareMap);
//        lift = new Lift(hardwareMap);

        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");
        sensorRangeM = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_middle");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");
    }

    @Override
    public void loop() {
        //telemetry
        //sensorTelemetry();
        drive.telemetry(telemetry);
        catapult.telemetry(telemetry);
//        carousel.telemetry(telemetry);
//        intake.telemetry(telemetry);
//        lift.telemetry(telemetry);
        telemetry.update();

        //DRIVER ASSIST
        //drive to shipping hub position
        if ((sensorRangeM.getDistance(DistanceUnit.INCH) <= 11.5 || sensorRangeM.getDistance(DistanceUnit.INCH) >= 12.5) &&
             gamepad1.dpad_up && sensorRangeM.getDistance(DistanceUnit.INCH) < 100) {
            drive.drive(0, (sensorRangeM.getDistance(DistanceUnit.INCH) - 12) / 10, 0);
        } else {
            drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x / 2); //drive function
        }

        //turns off surgical intake when catapult is moving back to the default position
        if (catapult.getVelocity() < 0) {
            surgicalToggle = false;
        }



        //CATAPULT BODY
        if ((gamepad2.y || gamepad1.y) && !lastPressedCatapultUpper) {
            catapultUpperToggle = !catapultUpperToggle;
            catapultMiddleToggle = false;
            catapultLowerToggle = false;
        } else if ((gamepad2.b || gamepad1.b)  && !lastPressedCatapultMiddle) {
            catapultMiddleToggle = !catapultMiddleToggle;
            catapultUpperToggle = false;
            catapultLowerToggle = false;
        } else if ((gamepad2.a  || gamepad1.a) && !lastPressedCatapultLower) {
            catapultLowerToggle = !catapultLowerToggle;
            catapultUpperToggle = false;
            catapultMiddleToggle = false;
        }
        if (catapultUpperToggle) {
            catapult.upper();
        } else if (catapultMiddleToggle) {
            catapult.middle();
        } else if (catapultLowerToggle) {
            catapult.lower();
        } else {
            catapult.returnPosition();
        }
        lastPressedCatapultUpper = gamepad2.y || gamepad1.y;
        lastPressedCatapultMiddle = gamepad2.b || gamepad1.b;
        lastPressedCatapultLower = gamepad2.a || gamepad1.a;

        //CATAPULT FLAP
//        if (((gamepad2.left_trigger > 0) || (gamepad1.left_trigger > 0)) && !lastPressedFlap) {
//            flapToggle = !flapToggle;
//        }
//        if (flapToggle) {
//            catapult.flapOpen();
//        } else {
//            catapult.flapClose();
//        }
//        lastPressedFlap = (gamepad2.left_trigger > 0) || (gamepad1.left_trigger > 0);

        //CATAPULT HEAD
//        if ((gamepad2.dpad_down || gamepad1.dpad_down) && !lastPressedCatapultHeadLeft) {
//            catapultHeadLeftToggle = !catapultHeadLeftToggle;
//        }
//        if (catapultHeadLeftToggle) {
//            catapult.headFold();
//        } else {
//            catapult.headUnfold();
//        }
//        lastPressedCatapultHeadLeft = gamepad2.dpad_down || gamepad1.dpad_down;

        //TEAM SHIPPING ELEMENT LIFT
//        if (gamepad2.left_bumper && !lastPressedLiftMotor) {
//            liftMotorToggle = !liftMotorToggle;
//        }
//        if (liftMotorToggle) {
//            lift.lift();
//        } else {
//            lift.lower();
//        }
//        lastPressedLiftMotor = gamepad2.left_bumper;
//
//        if (gamepad2.left_trigger > 0 && !lastPressedLiftServo) {
//            liftServoToggle = !liftServoToggle;
//        }
//        if (liftMotorToggle) {
//            lift.open();
//        } else {
//            lift.close();
//        }
//        lastPressedLiftServo = (gamepad2.left_trigger > 0);

        //SURGICAL TUBING INTAKE
//        if ((gamepad2.left_bumper || gamepad1.left_bumper) && !lastPressedSurgical) {
//            surgicalToggle = !surgicalToggle;
//        }
//        if (surgicalToggle) {
//            intake.surgicalTubingOn();
//        } else {
//            intake.surgicalTubingOff();
//        }
//        lastPressedSurgical = gamepad2.left_bumper || gamepad1.left_bumper;

        //INTAKE HOLDER
//        if((gamepad2.right_trigger > 0 || gamepad1.right_trigger > 0) && !lastPressedIntakeHolder) {
//            intakeHolderToggle = !intakeHolderToggle;
//        }
//        if(intakeHolderToggle) {
//            intake.dropIntake();
//        } else {
//            intake.returnIntakeHolder();
//        }
//        lastPressedIntakeHolder = (gamepad2.right_trigger > 0 || gamepad1.right_trigger > 0);

        //CAROUSEL SPINNER
//        if(gamepad2.dpad_left || gamepad1.dpad_left) {
//            carousel.spinRed();
//        } else if(gamepad2.dpad_right || gamepad1.dpad_right) {
//            carousel.spinBlue();
//        } else {
//            carousel.spinOff();
//        }
    }

    public void sensorTelemetry() {
        telemetry.addData("Left 2MDistance Range", String.format("%.01f in", sensorDistanceL.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Middle Range Range", String.format("%.01f in", sensorRangeM.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Right 2MDistance Range", String.format("%.01f in", sensorDistanceR.getDistance(DistanceUnit.INCH)));
    }
}
