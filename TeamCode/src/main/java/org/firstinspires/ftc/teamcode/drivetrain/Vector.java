package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.highlevel.Master.invSqrt;

public class Vector {
    // Current issue: references for all objects within MecanumDrive not working
    // Note that arrays are given by their reference address
    
    private double [] vectorArray;

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

    public void set(Vector vector){
        for(int i = 0; i < vector.getVector().length; i++){
            this.set(i, vector.getVector()[i]);
        }
    }

    public Vector neg(Vector vector){
        for(double thisNum : vector.getVector()){
            thisNum = -thisNum;
        }

        return this;
    }

    public Vector add(Vector vector){
        if(vector.getVector().length == this.getVector().length){
            for(int i = 0; i < vector.getVector().length; i++) {
                this.set(i, this.getVector()[i] + vector.getVector()[i]);
            }
        }

        return this;
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

    public Vector multiply(double coeff){
        for(int i = 0; i < this.vectorArray.length; i++){
            this.set(i, this.vectorArray[i] * coeff);
        }

        return this;
    }

    public double length(){
        double length = 0.0;

        for(double thisNum : this.getVector()){
            length += thisNum * thisNum;
        }

        return (Math.sqrt(length));
    }

    public Vector normalize(){
        double length = this.length();

        for(int i = 0; i < this.getVector().length; i++){
            this.set(i, invSqrt(this.getVector()[i]));
        }

        return this;
    }

    public Vector rotate(double angleChange){ // Radians
        this.set(0, Math.cos(angleChange) * this.getVector()[0] - Math.sin(angleChange) * this.getVector()[1]);
        this.set(1, Math.sin(angleChange) * this.getVector()[0] + Math.cos(angleChange) * this.getVector()[1]);

        return this;
    }

    /*public Vector multiply(Matrix matrix){
        double [] newVector = new double [this.getVector().length];

        for(int i = 0; i < newVector.length; i++){
            this.set(i, matrix.row(i).dot(this));
        }

        return this;
    }*/
}

