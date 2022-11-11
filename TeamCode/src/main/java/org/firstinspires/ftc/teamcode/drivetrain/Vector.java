package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.highlevel.Master.invSqrt;
import static org.firstinspires.ftc.teamcode.highlevel.Master.auxillaryNumber;
import static org.firstinspires.ftc.teamcode.highlevel.Master.auxillary;

import org.firstinspires.ftc.teamcode.highlevel.Master;

public class Vector { // Vector-based methods deprecated, use double []
    // Current issue: references for all objects within MecanumDrive not working
    // Note that arrays are given by their reference address
    
    private double [] vectorArray;
    private double auxillary = 0;

    public Vector(double [] vector){
        this.vectorArray = vector;
    }

    public Vector(Vector vector){
        this.set(0, vector.getVector()[0]);
        this.set(1, vector.getVector()[1]);
    }

    public double [] getVector() {
        return this.vectorArray;
    }

    public void set(int i, double num){
        this.vectorArray[i] = num;
    }

    public static double [] set(double [] list, int i, double num){
        list [i] = num;
        return list;
    }

    public Vector neg(Vector vector){
        for(double thisNum : vector.getVector()){
            thisNum = -thisNum;
        }

        return this;
    }

    public static double [] neg(double [] list){
        for(double i : list){
            i = -i;
        }
        return list;
    }

    public Vector add(Vector vector){
        if(vector.getVector().length == this.getVector().length){
            for(int i = 0; i < vector.getVector().length; i++) {
                this.set(i, this.getVector()[i] + vector.getVector()[i]);
            }
        }

        return this;
    }

    public static double [] add(double [] list1, double [] list2){ // List 1 altered
        if(list1.length == list2.length){
            for(int i = 0; i < list1.length; i++){
                Master.auxillary[i] = 0.0;
                Master.auxillary[i] = list1[i] +  list2[i];
            }
            return Master.auxillary;
        }
        return null;
    }

    public double dot(Vector vector){
        int total = 0;

        if(vector.getVector().length == this.getVector().length){
            for(int i = 0; i < vector.getVector().length; i++){
                total += this.getVector()[i] * vector.getVector()[i];
            }
        }

        return total;
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

    public Vector multiply(double coeff){
        for(int i = 0; i < this.vectorArray.length; i++){
            this.set(i, this.vectorArray[i] * coeff);
        }

        return this;
    }

    public static double [] multiply(double coeff, double [] list){
        for(double i : list){
            i *= coeff;
        }
        return list;
    }

    public double length(){
        double length = 0.0;

        for(double thisNum : this.getVector()){
            length += thisNum * thisNum;
        }

        return (Math.sqrt(length));
    }

    public static double lengthOf(double [] list){ // Alters list
        auxillaryNumber = 0;
        for(double i : list){
            auxillaryNumber += i * i;
        }
        return auxillaryNumber;
    }

    public Vector normalize(){
        double length = this.length();

        auxillary = 0;
        for(int i = 0; i < this.vectorArray.length; i++){
            auxillary += this.vectorArray[i] * this.vectorArray[i];
        }

        for(int i = 0; i < this.vectorArray.length; i++){
            this.vectorArray[i] *= invSqrt(vectorArray[i]);
        }

        return this;
    }

    public static double [] normalize(double [] list){
        auxillaryNumber = 0;
        auxillaryNumber = lengthOf(list);
        for(double i : list){
            i *= invSqrt(auxillaryNumber);
        }
        return list;
    }

    public Vector rotate(double angleChange){ // Radians
        this.set(0, Math.cos(angleChange) * this.getVector()[0] - Math.sin(angleChange) * this.getVector()[1]);
        this.set(1, Math.sin(angleChange) * this.getVector()[0] + Math.cos(angleChange) * this.getVector()[1]);

        return this;
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

