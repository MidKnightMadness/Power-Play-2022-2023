package org.firstinspires.ftc.teamcode.practicerobots.Demo;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.practicerobots.FreightFrenzy15385.Carousel;

// goofy ahh demo
// start by looking at this class then look at other subassembly classes

@TeleOp(name="Demo", group="Practice Robots") // signifies this is driver controlled
@Disabled // disables it from appearing in the menu (on the controller)
public class DemoTeleOp extends OpMode { // Uses OpMode for driver-controlled
    SampleDrive wheels; // create object of classes for subassemblies
    Intake intake;
    Lift lift;
    Servo servo;

    private DistanceSensor sensorDistance;
    private ModernRoboticsI2cRangeSensor sensorRange;

    private boolean lastPressedIntakeMotor = false;
    private boolean intakeMotorToggle = false;
    private boolean lastPressedLiftMotor = false;
    private boolean liftMotorToggle = false;

    // INIT METHOD
    // This method runs once after pressing INIT
    @Override // overrides a previous init method from OpMode
    public void init() {
        wheels = new SampleDrive(hardwareMap); // initialize objects
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        servo = new Servo(hardwareMap);

        //distance sensors
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance"); // 2 meter distance sensor
        sensorRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range"); // range sensor can detect much farther
    }


    // INIT_LOOP METHOD
    // This method runs repeatedly after pressing INIT
    public void init_loop() {

    }


    // START METHOD
    // This method runs once after pressing play
    public void start() {

    }


    // LOOP METHOD
    // This method runs repeatedly after pressing play
    @Override // overrides a previous loop method from OpMode
    public void loop() {

        //DRIVE
        wheels.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x); //drive function
        // left stick horizontal movement, left stick vertical movement, right stick horizontal movement


        //TOGGLE EXAMPLE - turning the motor to a specific position
        // when the button is pressed once, the motor moves to a specific position
        // when the button is pressed again, the motor moves back to the original position
        if (gamepad2.left_bumper && !lastPressedLiftMotor) {
            liftMotorToggle = !liftMotorToggle;
        }
        if (liftMotorToggle) {
            lift.lift();
        } else {
            lift.lower();
        }
        lastPressedLiftMotor = gamepad2.left_bumper;

        //TOGGLE EXAMPLE (INTAKE) - turning the motor constantly
        // when the button is pressed once, the motor constantly spins
        // when the button is pressed again, the motor stops spinning
        if ((gamepad2.right_bumper || gamepad1.right_bumper) && !lastPressedIntakeMotor) {
            intakeMotorToggle = !intakeMotorToggle;
        }
        if (intakeMotorToggle) {
            intake.surgicalTubingOn();
        } else {
            intake.surgicalTubingOff();
        }
        lastPressedIntakeMotor = gamepad2.left_bumper || gamepad1.left_bumper;

        //SERVO EXAMPLE - a continuous rotation servo that spins when the button is held
        if(gamepad2.dpad_left || gamepad1.dpad_left) {
            servo.spinRed(); // spins counter clockwise
        } else if(gamepad2.dpad_right || gamepad1.dpad_right) {
            servo.spinBlue(); // spins clockwise
        } else {
            servo.spinOff(); // else do nothing
        }


        //TELEMETRY
        sensorTelemetry();
        wheels.telemetry(telemetry);
        intake.telemetry(telemetry);
        lift.telemetry(telemetry);
        telemetry.update();
    }

    public void sensorTelemetry() {
        telemetry.addData("2MDistance Range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Range Range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
    }
}
