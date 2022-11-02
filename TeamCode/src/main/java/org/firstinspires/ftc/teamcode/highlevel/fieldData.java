package org.firstinspires.ftc.teamcode.highlevel;

import org.firstinspires.ftc.teamcode.odometry.Vector2;

public interface fieldData {
    final static double[][] junctionHeights = { // Matrix format, A[i][j], ith column, jth row; inches
            { 1.0,   14.5,  1.0,   14.5,  1.0 },

            { 14.5,  24.5,  34.5,  24.5,  14.5 },

            { 1.0,   34.5,  1.0,   34.5,  1.0 },

            { 14.5,  24.5,  34.5,  24.5,  14.5 },

            { 1.0,   14.5,  1.0,   14.5,  1.0 },

    };

    final int aposx = 60;
    final int fposx = 84;

    public static Vector2[][] signalLocations = new Vector2[][] {
        { new Vector2(aposx, 60), new Vector2(aposx, 36), new Vector2(aposx, 12) },
        { new Vector2(aposx, 132), new Vector2(aposx, 108), new Vector2(aposx, 84) },
        { new Vector2(fposx, 12), new Vector2(fposx, 36), new Vector2(fposx, 60) },
        { new Vector2(fposx, 132), new Vector2(fposx, 108), new Vector2(fposx, 84) },
    };
}
