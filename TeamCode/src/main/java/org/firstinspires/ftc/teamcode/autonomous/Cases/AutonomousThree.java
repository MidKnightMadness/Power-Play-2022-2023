package org.firstinspires.ftc.teamcode.autonomous.Cases;

import org.firstinspires.ftc.teamcode.autonomous.Autonomous;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousThree extends Autonomous {
    @Override
    public int getStartingPos() {
        return 2;
    }


    public double getStartingRotation() {
        return -Math.PI / 2;
    }

    public Vector2 getStartingPostition() {
        return new Vector2(realSquareWidth * 6 - halfRobotWidth, realSquareWidth * 4.5);
    }
}
