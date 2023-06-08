
package org.firstinspires.ftc.teamcode.drivetrain;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous
public class MotorCalibrator extends OpMode {
    DcMotorEx motor;

    double ticksPerSecond = 0;

    double gearRatio = 1 / 40;
    double ticksPerRotation = Math.abs(1120 * gearRatio);


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start(){
//        telemetry.addData("Initial motor ticks", motor.getCurrentPosition());
//        motor.setTargetPosition((int) ticksPerRotation); // Will now measure degrees turned
//        motor.setPower(0.5);
//
//        while(motor.isBusy()){
//            telemetry.addData("Motor ticks", motor.getCurrentPosition());
//        }

        // Testing max speed
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(1.0);
//        while(true){
//            telemetry.addData("Motor config power", motor.getPower());
//            telemetry.addData("Motor max speed", motor.getVelocity());
//
//            telemetry.addData("Motor config current", motor.getCurrent(CurrentUnit.AMPS));
//        }
    }


    double motorConfigPower = 0.0;

    @Override
    public void loop() {
//        motorConfigPower += 0.01;
//
//        sleep(100);
    }
}
