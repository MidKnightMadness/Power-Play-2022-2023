package org.firstinspires.ftc.teamcode.highlevel;

import static org.firstinspires.ftc.teamcode.highlevel.Master.currentPosition;
import static org.firstinspires.ftc.teamcode.highlevel.Master.odometryAlg;
import static org.firstinspires.ftc.teamcode.highlevel.Master.turntableAngle;
import static org.firstinspires.ftc.teamcode.manipulator.LinearSlides.ROOT_HEIGHT;

import org.firstinspires.ftc.teamcode.drivetrain.Vector;
import org.firstinspires.ftc.teamcode.highlevel.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.manipulator.DoubleReverse4Bar;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.odometry.TestingOdometryAlgorithm;
import org.firstinspires.ftc.teamcode.drivetrain.*;


public class GridSystem implements fieldData{
    private static double [] auxillary3d; // Use currentPosition if needed, but should pre-calculate during transition
    private static double [] auxillary2d;
    
    public GridSystem(){
        auxillary3d = new double [] {0.0, 0.0, 0.0};
        auxillary2d = new double [] {0.0, 0.0};
    }
    
    final static double[][] junctionHeights = { // Matrix format, A[i][j], ith column, jth row; inches
            { 1.0,   14.5,  1.0,   14.5,  1.0 },

            { 14.5,  24.5,  34.5,  24.5,  14.5 },

            { 1.0,   34.5,  1.0,   34.5,  1.0 },

            { 14.5,  24.5,  34.5,  24.5,  14.5 },

            { 1.0,   14.5,  1.0,   14.5,  1.0 },

    };

    public static double [] closestJunctionDisplacement(Vector position){ // Returns 3d displacement vector to be inputed to move methods
        odometryAlg.updateOrientationAndLocation();
        auxillary2d = position.getVector();

        auxillary3d[0] = auxillary2d[0] - (23.50 * (Math.round(auxillary2d[0] / 23.50)));
        auxillary3d[1] = auxillary2d[1] - (23.50 * (Math.round(auxillary2d[1] / 23.50)));
        auxillary3d[2] = junctionHeights[(int) (6 - Math.round(auxillary2d[0] / 23.50))]
                [(int) (Math.round(auxillary2d[1] / 23.50) - 1)] - ROOT_HEIGHT;
        
        return auxillary3d;
    } // Ready for input into "pointAt"

    public static double [] pointAtJunction(Vector position){ // Returns 3d array with hor. angle change, vert. angle change, and extension length
        odometryAlg.updateOrientationAndLocation();
        auxillary2d = position.getVector();

        auxillary2d[0] %= 23.50;
        auxillary2d[1] %= 23.50;

        //
        return new double[] {4d, 5d};


        // Floor with type casts for some of these
    }
}


