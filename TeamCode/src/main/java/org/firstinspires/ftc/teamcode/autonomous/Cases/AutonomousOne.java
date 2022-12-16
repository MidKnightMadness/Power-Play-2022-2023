package org.firstinspires.ftc.teamcode.autonomous.Cases;

import org.firstinspires.ftc.teamcode.autonomous.Autonomous;
import org.firstinspires.ftc.teamcode.odometry.Vector2;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousOne extends Autonomous {
    @Override
    public int getStartingPos() {
        return 1;
    }

    public double getStartingRotation() {
        return Math.PI / 2;
    }

    public Vector2 getStartingPosition() {
        return new Vector2(35, halfRobotWidth);
    }
}
