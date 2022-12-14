package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.highlevel.Master.invSqrt;
import static org.firstinspires.ftc.teamcode.highlevel.Master.auxillaryNumber;

import org.firstinspires.ftc.teamcode.highlevel.Master;

import java.util.*;
import java.math.*;

@Deprecated
public class Vector { // Vector-based methods deprecated, use double []
    // Current issue: references for all objects within MecanumDrive not working
    // Note that arrays are given by their reference address
    public static double [] auxillary = {0.0, 0.0, 0.0, 0.0};
    public static double auxillaryNumber = 0.0;

    public static double [] set(double [] list, int i, double num){
        list [i] = num;
        return list;
    }

    public static double [] neg(double [] list){
        for(int i = 0; i < list.length; i++){

            auxillary[i] = -list[i];
        }
        return auxillary;
    }

    public static double [] add(double [] list1, double [] list2){ // List 1 altered
//        if(list1.length == list2.length){
//            for(int i = 0; i < list1.length; i++){
//                Master.auxillary[i] = 0.0;
//                Master.auxillary[i] = list1[i] +  list2[i];
//            }
//            return Master.auxillary;
//        }
        for(int i = 0; i < list1.length; i++){
            auxillary[i] = list1[i];
            auxillary[i] += list2[i];
        }
        return auxillary;
    }

    public static double dot(double [] list1, double [] list2){
        if(list1.length == list2.length){
            auxillaryNumber = 0.0;
            for(int i = 0; i < list1.length; i++){
                auxillaryNumber += list1[i] * list2[i];
            }
            return auxillaryNumber;
        }
        return -1.0;
    }

    public static double [] multiply(double coeff, double [] list){
        for(int i = 0; i < list.length; i++){
            auxillary[i] = list[i];
            auxillary[i] *= coeff;
        }
        return auxillary;
    }

    public static double lengthOf(double [] list){ // Alters list
        auxillaryNumber = 0;
        for(double i : list){
            auxillaryNumber += i * i;
        }
        return Math.sqrt(auxillaryNumber);
    }

    public static double [] normalize(double [] list){
        for(int i = 0; i < list.length; i++){
            list[i] *= Master.invSqrt(lengthOf(list));
        }
        return list;
    }

    public static double [] rotateBy(double [] list, double angleChange){ // Radians
        auxillary[0] = list[0] * Math.cos(angleChange) - list[1] * Math.sin(angleChange);
        auxillary[1] = list[0] * Math.sin(angleChange) + list[1] * Math.cos(angleChange);
        return auxillary;
    }

    public static boolean areEqual(double [] list1, double [] list2){ // Checks if they're equal
        if(list1.length == list2.length){
            for(int i = 0; i < list1.length; i++){
                if(list1[i] != list2[i]){
                    return false;
                }
            }
            return true;
        }
        return false;
    }

    public static double [] equalTo(double [] list1, double [] list2){ // Copies data of 2nd list to 1st list
        for(int i = 0; i < Math.min(list1.length, list2.length); i++){
            list1[i] = list2[i];
        }
        return list1;
    } // If this doesn't work, copy all the data manually


    /*public Vector multiply(Matrix matrix){
        double [] newVector = new double [this.getVector().length];

        for(int i = 0; i < newVector.length; i++){
            this.set(i, matrix.row(i).dot(this));
        }

        return this;
    }*/
}



