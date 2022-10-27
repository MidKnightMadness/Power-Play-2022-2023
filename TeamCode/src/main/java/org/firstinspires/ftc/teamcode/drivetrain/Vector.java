package org.firstinspires.ftc.teamcode.drivetrain;

public class Vector {
    private double [] vector;

    public Vector(double [] vector){
        this.vector = vector;
    }

    public Vector(Vector vector){
        this.set(0, vector.get()[0]);
        this.set(1, vector.get()[1]);
    }

    public double [] get(){
        return this.vector;
    }

    public void set(int i, double num){
        this.vector[i] = num;
    }

    public void set(Vector vector){
        for(int i = 0; i < vector.get().length; i++){
            this.set(i, vector.get()[i]);
        }
    }

    public Vector neg(Vector vector){
        for(double thisNum : vector.get()){
            thisNum = -thisNum;
        }

        return this;
    }

    public Vector add(Vector vector){
        if(vector.get().length == this.get().length){
            for(int i = 0; i < vector.get().length; i++) {
                this.set(i, this.get()[i] + vector.get()[i]);
            }
        }

        return this;
    }

    public double dot(Vector vector){
        int total = 0;

        if(vector.get().length == this.get().length){
            for(int i = 0; i < vector.get().length; i++){
                total += this.get()[i] * vector.get()[i];
            }
        }

        return total;
    }

    public Vector multiply(double coeff){
        for(int i = 0; i < this.vector.length; i++){
            this.set(i, this.vector[i] * coeff);
        }

        return this;
    }

    public double length(){
        double length = 0.0;

        for(double thisNum : this.get()){
            length += thisNum * thisNum;
        }

        return (Math.sqrt(length));
    }

    public Vector normalize(){
        double length = this.length();

        for(int i = 0; i < this.get().length; i++){
            this.set(i, this.get()[i] / length);
        }

        return this;
    }

    public Vector rotate(double angleChange){ // Radians
        this.set(0, Math.cos(angleChange) * this.get()[0] - Math.sin(angleChange) * this.get()[1]);
        this.set(1, Math.sin(angleChange) * this.get()[0] + Math.cos(angleChange) * this.get()[1]);

        return this;
    }

    /*public Vector multiply(Matrix matrix){
        double [] newVector = new double [this.get().length];

        for(int i = 0; i < newVector.length; i++){
            this.set(i, matrix.row(i).dot(this));
        }

        return this;
    }*/
}

