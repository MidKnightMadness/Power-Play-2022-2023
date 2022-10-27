package org.firstinspires.ftc.teamcode.highlevel;

import static org.firstinspires.ftc.teamcode.highlevel.Master.currentPosition;
import static org.firstinspires.ftc.teamcode.highlevel.Master.odometryAlg;
import static org.firstinspires.ftc.teamcode.manipulator.LinearSlides.ROOT_HEIGHT;

import org.firstinspires.ftc.teamcode.drivetrain.Vector;
import org.firstinspires.ftc.teamcode.highlevel.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.manipulator.DoubleReverse4Bar;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.odometry.TestingOdometryAlgorithm;
import org.firstinspires.ftc.teamcode.drivetrain.*;


public class GridSystem {
    final static double[][] junctionHeights = { // Matrix format, A[i][j], ith column, jth row; inches
            { 1.0,   14.5,  1.0,   14.5,  1.0 },

            { 14.5,  24.5,  34.5,  24.5,  14.5 },

            { 1.0,   34.5,  1.0,   34.5,  1.0 },

            { 14.5,  24.5,  34.5,  24.5,  14.5 },

            { 1.0,   14.5,  1.0,   14.5,  1.0 },

    };

    public static double [] closestJunctionDisplacement(Vector position){ // Returns 3d displacement vector to be inputed to move methods
        odometryAlg.updateOrientationAndLocation();
        double [] auxillaryArrayToSpeedUpStuff = position.get(); // Use currentPosition if needed, but should pre-calculate during transition

        return new double [] {auxillaryArrayToSpeedUpStuff[0] - (22.75 * (Math.round(auxillaryArrayToSpeedUpStuff[0] / 22.75))),
                auxillaryArrayToSpeedUpStuff[1] - (22.75 * (Math.round(auxillaryArrayToSpeedUpStuff[1] / 22.75))),
                junctionHeights[(int) (6 - Math.round(auxillaryArrayToSpeedUpStuff[0] / 22.75))]
                        [(int) (Math.round(auxillaryArrayToSpeedUpStuff[1] / 22.75) - 1)] - ROOT_HEIGHT};
    }
}


