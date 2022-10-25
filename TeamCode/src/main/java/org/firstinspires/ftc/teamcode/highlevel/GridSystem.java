package org.firstinspires.ftc.teamcode.highlevel;

import org.firstinspires.ftc.teamcode.drivetrain.Vector;

enum Junction {
    Ground,
    Low,
    Medium,
    High
}

public class GridSystem {
    final Junction[][] junctionPositions = {
            { Junction.Ground, Junction.Low, Junction.Ground, Junction.Low, Junction.Ground },
            { Junction.Low, Junction.Medium, Junction.High, Junction.Medium, Junction.Low },
            { Junction.Ground, Junction.High, Junction.Ground, Junction.High, Junction.Ground },
            { Junction.Low, Junction.Medium, Junction.High, Junction.Medium, Junction.Low },
            { Junction.Ground, Junction.Low, Junction.Ground, Junction.Low, Junction.Ground },
    };

    final Integer[][] positions = {
            { 0, 1, 0, 1, 0 } ,
            { 1, 2, 3, 2, 1 },
            { 0, 3, 0, 3, 0 },
            { 1, 2, 3, 2, 1 },
            { 0, 1, 0, 1, 0 },
    };

    Vector position = new Vector();
    double rotation;







}
