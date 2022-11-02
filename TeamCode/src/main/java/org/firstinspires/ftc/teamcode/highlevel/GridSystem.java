package org.firstinspires.ftc.teamcode.highlevel;

import static org.firstinspires.ftc.teamcode.highlevel.Master.odometryAlg;
import static org.firstinspires.ftc.teamcode.manipulator.LinearSlides.ROOT_HEIGHT;

import org.firstinspires.ftc.teamcode.drivetrain.Vector;


public class GridSystem implements fieldData {

    public static double [] closestJunctionDisplacement(Vector position){ // Returns 3d displacement vector to be inputed to move methods
        odometryAlg.updateOrientationAndLocation();
        double [] auxillaryArrayToSpeedUpStuff = position.getVector(); // Use currentPosition if needed, but should pre-calculate during transition

        return new double [] {auxillaryArrayToSpeedUpStuff[0] - (23.50 * (Math.round(auxillaryArrayToSpeedUpStuff[0] / 23.50))),
                auxillaryArrayToSpeedUpStuff[1] - (23.50 * (Math.round(auxillaryArrayToSpeedUpStuff[1] / 23.50))),
                junctionHeights[(int) (6 - Math.round(auxillaryArrayToSpeedUpStuff[0] / 23.50))]
                        [(int) (Math.round(auxillaryArrayToSpeedUpStuff[1] / 23.50) - 1)] - ROOT_HEIGHT};
    }
}


