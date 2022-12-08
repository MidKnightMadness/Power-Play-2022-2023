package org.firstinspires.ftc.teamcode.highlevel;

import static org.firstinspires.ftc.teamcode.highlevel.Master.MAX_SCORING_RADIUS;
import static org.firstinspires.ftc.teamcode.highlevel.Master.auxillary;
import static org.firstinspires.ftc.teamcode.highlevel.Master.currentPosition;
import static org.firstinspires.ftc.teamcode.highlevel.Master.invSqrt;
import static org.firstinspires.ftc.teamcode.highlevel.Master.odometryAlg;
import static org.firstinspires.ftc.teamcode.highlevel.Master.robotSpeed;
import static org.firstinspires.ftc.teamcode.highlevel.Master.tickRate;
import static org.firstinspires.ftc.teamcode.highlevel.Master.turntable;
import static org.firstinspires.ftc.teamcode.manipulator.LinearSlides.ROOT_HEIGHT;

import org.firstinspires.ftc.teamcode.currentOpModes.MainTeleOp;
import org.firstinspires.ftc.teamcode.highlevel.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.manipulator.DoubleReverse4Bar;
import org.firstinspires.ftc.teamcode.manipulator.LinearSlides;
import org.firstinspires.ftc.teamcode.manipulator.Turntable;
import org.firstinspires.ftc.teamcode.odometry.TestingOdometryAlgorithm;
import org.firstinspires.ftc.teamcode.drivetrain.*;



public class GridSystem {
    private static double [] auxillary3d = {0.0, 0.0, 0.0}; // Use currentPosition if needed, but should pre-calculate during transition
    private static double [] auxillary = {0.0, 0.0, 0.0};

    // 2 "local" variables to deal with internal calculations
    private static double djskjsjksskjkj = 0.0;
    private static int djskjsjksskjkj2 = 0;
    
    public GridSystem(){
    }
    
    final static double[][] junctionHeights = { // Matrix format, A[i][j], ith column, jth row; inches
            // *12-6-22 Mike added 3.0 in. to all junction heights for position of end of arm
            // (shorter than claw effective distance)

            { 4.0,   17.5,  4.0,   17.5,  4.0 }, // 0,0 at top left ig for convenience due to symmetry

            { 17.5,  27.5,  37.5,  27.5,  17.5 },

            { 4.0,   37.5,  4.0,   37.5,  4.0 },

            { 17.5,  27.5,  37.5,  27.5,  17.5 },

            { 4.0,   17.5,  4.0,   17.5,  4.0 },

    };

    public static double [] closestJunctionDisplacement(double currentX, double currentY){ // Returns 3d displacement vector to be inputed to move methods
        odometryAlg.updateOrientationAndLocation();
        double [] auxillary2d = {currentX, currentY}; // Change

        auxillary3d[0] = auxillary2d[0] - (23.50 * (Math.round(auxillary2d[0] / 23.50)));
        auxillary3d[1] = auxillary2d[1] - (23.50 * (Math.round(auxillary2d[1] / 23.50)));
        auxillary3d[2] = junctionHeights[(int) (Math.round(auxillary2d[0] / 23.50) - 1)]
                [(int) (Math.round(auxillary2d[1] / 23.50) - 1)] - ROOT_HEIGHT;
        
        return auxillary3d;
    } // Ready for input into "pointAt"

    // Revise for backwards scoring!
    // Let's probably modify this into something to input into the pointAt function, still relative to center of original turntable
    public static double [] pointAtJunction(double currentX, double currentY){ // Returns 3d array with hor. angle target, vert. angle target, and extension length
        MainTeleOp.odometry.updatePosition();
        double [] auxillary2d = {currentX, currentY};
        // Holding down a key continuously aims at the same junction, with small adjustments to compensate
        // This method just locks onto a junction

        auxillary2d[0] %= 23.50;
        auxillary2d[1] %= 23.50;
        djskjsjksskjkj = 0.0;
        djskjsjksskjkj2 = 1; // Starts from 1, counts to 4th junction

        // Check each junction in square, check if inside max scoring radius
        // Note that with scoring radius, checking everything in current square will work, even on boundaries
        // Top right
        auxillary2d[0] = 23.50 - auxillary2d[0];
        auxillary2d[1] = 23.50 - auxillary2d[1];
        if(Math.sqrt(auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1]) < MAX_SCORING_RADIUS){
            djskjsjksskjkj = Math.atan(auxillary2d[1] / auxillary2d[0]);
        }

        // Top left
        auxillary2d[0] -= 23.50;
        if(Math.sqrt(auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1]) < MAX_SCORING_RADIUS &&
                odometryAlg.orientationAngle - Math.PI - Math.atan(auxillary2d[1] / auxillary2d[0]) < djskjsjksskjkj){
            djskjsjksskjkj = Math.PI + Math.atan(auxillary2d[1] / auxillary2d[0]);
            djskjsjksskjkj2 = 2;
        }

        // Bottom left
        auxillary2d[1] -= 23.50;
        if(Math.sqrt(auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1]) < MAX_SCORING_RADIUS &&
                odometryAlg.orientationAngle - Math.atan(auxillary2d[1] / auxillary2d[0]) < djskjsjksskjkj){
            djskjsjksskjkj = Math.PI + Math.atan(auxillary2d[1] / auxillary2d[0]);
            djskjsjksskjkj2 = 3;
        }

        // Bottom right
        auxillary2d[0] += 23.50;
        if(Math.sqrt(auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1]) < MAX_SCORING_RADIUS &&
                odometryAlg.orientationAngle - Math.PI - Math.atan(auxillary2d[1] / auxillary2d[0]) < djskjsjksskjkj){
            djskjsjksskjkj = (2 * Math.PI) + Math.atan(auxillary2d[1] / auxillary2d[0]);
            djskjsjksskjkj2 = 4;
        }

        auxillary3d[0] = djskjsjksskjkj - MainTeleOp.odometry.getRotationRadians(); // Turntable target angle

        // "Points" at junction corresponding to target position to get required height and extension length
        auxillary2d[0] = currentX;
        auxillary2d[1] = currentY;
        switch(djskjsjksskjkj2){
            // Junction numbering in square
            // 2 1
            // 3 4
            case 1:
                // (23.50, 23.50) relative coords
                auxillary2d[0] = (int) (auxillary2d[0] / 23.50); // Floors to highest x and y coordinates in box, then converts into list index by subtracting 1
                auxillary2d[1] = (int) (auxillary2d[1] / 23.50);

                if(auxillary[0] < 5 && auxillary2d[1] < 5){
                    djskjsjksskjkj = junctionHeights [(int) auxillary2d[0]][(int) auxillary2d[1]] - ROOT_HEIGHT;
                }else{
                    auxillary3d[1] = 0.0;
                    auxillary3d[2] = 0.0;
                    break;
                }

                // Making auxillary2d displacement to junction
                auxillary2d[0] = currentX;
                auxillary2d[1] = currentY;
                auxillary2d[0] = 23.50 - (auxillary2d[0] % 23.50);
                auxillary2d[1] = 23.50 - (auxillary2d[1] % 23.50);
                Master.target = auxillary2d;

                auxillary3d[1] = Math.atan(djskjsjksskjkj * invSqrt((auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1]))); // Angle, note that this doesn't go beyond 90˚, no need to flip
                auxillary3d[2] = Math.sqrt((djskjsjksskjkj * djskjsjksskjkj) + (auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1]));

                break;
            case 2:
                // (0.00 , 23.50) relative coords
                auxillary2d[0] = ((int) (auxillary2d[0] / 23.50)) - 1;
                auxillary2d[1] = (int) (auxillary2d[1] / 23.50);

                if(auxillary[0] < 5 && auxillary2d[1] < 5){
                    djskjsjksskjkj = junctionHeights [(int) auxillary2d[0]][(int) auxillary2d[1]] - ROOT_HEIGHT;
                }else{
                    auxillary3d[1] = 0.0;
                    auxillary3d[2] = 0.0;
                    break;
                }

                // Making auxillary2d displacement to junction
                auxillary2d[0] = currentX;
                auxillary2d[1] = currentY;
                auxillary2d[0] = auxillary2d[0] % 23.50;
                auxillary2d[1] = 23.50 - (auxillary2d[1] % 23.50);
                Master.target = auxillary2d;

                auxillary3d[1] = Math.atan(djskjsjksskjkj * invSqrt((auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1])));
                auxillary3d[2] = Math.sqrt((djskjsjksskjkj * djskjsjksskjkj) + (auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1]));

                break;
            case 3:
                // (0.00, 0.00) relative coords
                auxillary2d[0] = ((int) (auxillary2d[0] / 23.50)) - 1;
                auxillary2d[1] = ((int) (auxillary2d[1] / 23.50)) - 1;

                if(auxillary[0] < 5 && auxillary2d[1] < 5){
                    djskjsjksskjkj = junctionHeights [(int) auxillary2d[0]][(int) auxillary2d[1]] - ROOT_HEIGHT;
                }else{
                    auxillary3d[1] = 0.0;
                    auxillary3d[2] = 0.0;
                    break;
                }

                // Making auxillary2d displacement to junction
                auxillary2d[0] = currentX;
                auxillary2d[1] = currentY;
                auxillary2d[0] = auxillary2d[0] % 23.50;
                auxillary2d[1] = auxillary2d[1] % 23.50;
                Master.target = auxillary2d;

                auxillary3d[1] = auxillary3d[1] = Math.atan(djskjsjksskjkj * invSqrt((auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1])));
                auxillary3d[2] = Math.sqrt((djskjsjksskjkj * djskjsjksskjkj) + (auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1]));

                break;
            case 4:
                // (23.50, 0.00) relative coords
                auxillary2d[0] = (int) (auxillary2d[0] / 23.50);
                auxillary2d[1] = ((int) (auxillary2d[1] / 23.50)) - 1;

                if(auxillary[0] < 5 && auxillary2d[1] < 5){
                    djskjsjksskjkj = junctionHeights [(int) auxillary2d[0]][(int) auxillary2d[1]] - ROOT_HEIGHT;
                }else{
                    auxillary3d[1] = 0.0;
                    auxillary3d[2] = 0.0;
                    break;
                }

                // Making auxillary2d displacement to junction
                auxillary2d[0] = currentX;
                auxillary2d[1] = currentY;
                auxillary2d[0] = 23.50 - (auxillary2d[0] % 23.50);
                auxillary2d[1] = auxillary2d[1] % 23.50;
                Master.target = auxillary2d;

                auxillary3d[1] = auxillary3d[1] = Math.atan(djskjsjksskjkj * invSqrt((auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1])));
                auxillary3d[2] = Math.sqrt((djskjsjksskjkj * djskjsjksskjkj) + (auxillary2d[0] * auxillary2d[0]) + (auxillary2d[1] * auxillary2d[1]));

                break;
            default:
                break;
        }

        return auxillary3d;
    } // Still need to input to motors

    // Modified version for backwards scoring, outputs target angles and extensions in array
    static double [] scoringAdjustments(double currentX, double currentY, double targetX){
        return null;
    }
}


