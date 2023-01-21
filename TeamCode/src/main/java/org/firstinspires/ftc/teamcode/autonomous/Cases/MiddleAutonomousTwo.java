package org.firstinspires.ftc.teamcode.autonomous.Cases;

import org.firstinspires.ftc.teamcode.autonomous.Autonomous;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Right Middle")
public class MiddleAutonomousTwo extends Autonomous {
    @Override
    public int getStartingPos() {
        return 2;
    }

    @Override
    public double getStartingRotation() {
        return Math.PI / 2;
    }

    @Override
    public Vector2 getStartingPosition() {
        return new Vector2(104.5, halfRobotWidth);
    }

    @Override
    public int getScoringJunction() { return 1; }

}
