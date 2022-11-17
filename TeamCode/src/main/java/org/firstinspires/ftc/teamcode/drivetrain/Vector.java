package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.highlevel.Master.invSqrt;
import static org.firstinspires.ftc.teamcode.highlevel.Master.auxillaryNumber;

import org.firstinspires.ftc.teamcode.highlevel.Master;

public class Vector { // Vector-based methods deprecated, use double []
    // Current issue: references for all objects within MecanumDrive not working
    // Note that arrays are given by their reference address

    public static double [] set(double [] list, int i, double num){
        list [i] = num;
        return list;
    }

    public static double [] neg(double [] list){
        for(int i = 0; i < list.length; i++){
            list[i] = -list[i];
        }
        return list;
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
            Master.auxillary[i] = list1[i] +  list2[i];
        }
        return Master.auxillary;
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
            list[i] *= coeff;
        }
        return list;
    }

    public static double lengthOf(double [] list){ // Alters list
        auxillaryNumber = 0;
        for(double i : list){
            auxillaryNumber += i * i;
        }
        return Math.sqrt(auxillaryNumber);
    }

    public static double [] normalize(double [] list){
        auxillaryNumber = lengthOf(list);
        for(int i = 0; i < list.length; i++){
            list[i] *= invSqrt(auxillaryNumber);
        }
        return list;
    }

    public static double [] rotateBy(double [] list, double angleChange){ // Radians
        Master.auxillary[0] = list[0] * Math.cos(angleChange) - list[1] * Math.sin(angleChange);
        Master.auxillary[1] = list[0] * Math.sin(angleChange) + list[1] * Math.cos(angleChange);
        return list;
    }

    /*public Vector multiply(Matrix matrix){
        double [] newVector = new double [this.getVector().length];

        for(int i = 0; i < newVector.length; i++){
            this.set(i, matrix.row(i).dot(this));
        }

        return this;
    }*/
}

