package org.firstinspires.ftc.teamcode.autonomous.Cases;

import org.firstinspires.ftc.teamcode.autonomous.Autonomous;
import org.firstinspires.ftc.teamcode.odometry.Vector2;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Left Middle")
public class MiddleAutonomousOne extends Autonomous {
    @Override
    public int getStartingPos() {
        return 1;
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