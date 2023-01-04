package org.firstinspires.ftc.teamcode.autonomous.Cases;

import org.firstinspires.ftc.teamcode.autonomous.Autonomous;
import org.firstinspires.ftc.teamcode.odometry.Vector2;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Right")
public class AutonomousFour extends Autonomous {
    @Override
    public int getStartingPos() {
        return 4;
    }

    public double getStartingRotation() {
        return Math.PI / 2;
    }

    public Vector2 getStartingPosition() {
        return new Vector2(104.5, halfRobotWidth);
    }

}
