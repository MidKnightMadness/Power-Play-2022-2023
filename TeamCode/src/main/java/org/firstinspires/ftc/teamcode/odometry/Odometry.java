package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousNew;
import org.firstinspires.ftc.teamcode.currentOpModes.MainTeleOp;
import static org.firstinspires.ftc.teamcode.currentOpModes.MainTeleOp.currentPosition;

import java.util.concurrent.TimeUnit;

public class Odometry implements OdometryVariables {
    double deltaTime = 0;
    double lastTime = 0;

    Vector2 position = new Vector2();
    Vector2 velocity = new Vector2();

    int lastLeftTicks = 0;
    int deltaLeftTicks = 0;
    double leftDistanceMoved;

    int lastRightTicks = 0;
    int deltaRightTicks = 0;
    double rightDistanceMoved;

    int lastTopTicks = 0;
    int deltaTopTicks = 0;
    double topDistanceMoved;

    double rotationRadians;

    ElapsedTime elapsedTime;

    public DcMotorEx leftEncoder;
    public DcMotorEx horizontalEncoder;
    public DcMotorEx rightEncoder;

    public Odometry(HardwareMap hardwareMap) {
        elapsedTime = new ElapsedTime();

        leftEncoder = hardwareMap.get(DcMotorEx.class, "BL");
//        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        rightEncoder = hardwareMap.get(DcMotorEx.class, "FR");
//        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "BR");
//        horizontalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateTime() {
        double currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) / 1000000.0d;
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;
    }

    int leftTicks;
    int rightTicks;
    int topTicks;

    double deltaRadians;

    double forwardMovement;

    double lateralMovementAdjustor;
    double trueLateralMovement;

    double sin;
    double cosine;

    double netX;
    double netY;

    public void updatePosition() {
        leftTicks = leftEncoder.getCurrentPosition();
        rightTicks = -rightEncoder.getCurrentPosition();
        topTicks = horizontalEncoder.getCurrentPosition();

        deltaLeftTicks = leftTicks - lastLeftTicks;
        deltaRightTicks = rightTicks - lastRightTicks;
        deltaTopTicks = topTicks - lastTopTicks;

        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastTopTicks = topTicks;

        // raw distance from each encoder
        leftDistanceMoved = inPerTick * deltaLeftTicks;
        rightDistanceMoved = inPerTick * deltaRightTicks;
        topDistanceMoved = inPerTick * deltaTopTicks;

        // angles
        deltaRadians = getDeltaRotation(leftDistanceMoved, rightDistanceMoved);
        rotationRadians += deltaRadians;
//
        { // Simplified version
            netX += Math.cos(rotationRadians) * (0.5 * leftDistanceMoved + 0.5 * rightDistanceMoved);
            netY += Math.cos(rotationRadians) * (0.5 * leftDistanceMoved + 0.5 * rightDistanceMoved);
        }
        forwardMovement = (leftDistanceMoved + rightDistanceMoved) / 2.0;

        lateralMovementAdjustor = deltaRadians * verticalWheelDistance;
        trueLateralMovement = topDistanceMoved + lateralMovementAdjustor;

        sin = Math.sin(rotationRadians);
        cosine = Math.cos(rotationRadians);

        netX = forwardMovement * cosine + trueLateralMovement * sin;
        netY = forwardMovement * sin + trueLateralMovement * cosine;

        position.x -= netX;
        position.y -= netY;

        // Temporary
        AutonomousNew.currentPosition[0] += netX;
        AutonomousNew.currentPosition[1] += netY;

        MainTeleOp.currentPosition[0] += netX;
        MainTeleOp.currentPosition[1] += netY;

        velocity.x = netX / deltaTime;
        velocity.y = netY / deltaTime;
    }

    public double getDeltaRotation(double leftChange, double rightChange) {
        return (rightChange - leftChange) / lateralWheelDistance;
    }

    public double getXCoordinate() {
        return position.x;
    }

    public double getYCoordinate() {
        return position.y;
    }

    public double getRotationRadians() {
        return rotationRadians;
    }

    public double getRotationDegrees() {
        return rotationRadians * 180 / Math.PI;
    }

    public Vector2 getVelocity() {
        return velocity;
    }
    public void resetEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public String positionToString() {return String.format("(%f, %f)", position.x, position.y); }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("\nTHREE WHEEL ODOMETRY");

        telemetry.addLine(String.valueOf(deltaTime));

        telemetry.addData("Wheel ticks", String.format("%d, %d, %d", leftTicks, rightTicks, topTicks));
        telemetry.addData("Delta wheel ticks", String.format("%d, %d, %d", deltaLeftTicks, deltaRightTicks, deltaTopTicks));

        telemetry.addData("Movement", String.format("%f, %f", forwardMovement, trueLateralMovement));
        telemetry.addData("Net movement", String.format("%d, %d", deltaLeftTicks, deltaRightTicks));

        telemetry.addLine("Position " + position);
        telemetry.addLine("Velocity " + velocity.toString());
        telemetry.addLine(String.format("Rotation: %f", deltaRadians * 180 / Math.PI));
        telemetry.addLine("Rotation " + (rotationRadians * 180 / Math.PI));

        telemetry.addData("Left Dead Wheel Position", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Dead Wheel Position", rightEncoder.getCurrentPosition());
        telemetry.addData("Top Dead Wheel Position", horizontalEncoder.getCurrentPosition());

    }

}

