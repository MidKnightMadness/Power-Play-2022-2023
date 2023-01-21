package org.firstinspires.ftc.teamcode.autonomous.Cases;

import org.firstinspires.ftc.teamcode.autonomous.Autonomous;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Left Middle")
public class MiddleAutonomousThree extends Autonomous {
    @Override
    public int getStartingPos() {
        return 3;
    }
    @Override
    public double getStartingRotation() {
        return Math.PI / 2;
    }
    @Override
    public Vector2 getStartingPosition() {
        return new Vector2(35, halfRobotWidth);
    }

    @Override
    public int getScoringJunction() { return 1; }
}