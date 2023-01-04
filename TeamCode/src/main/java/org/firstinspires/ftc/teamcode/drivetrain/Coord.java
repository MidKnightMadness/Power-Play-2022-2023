package org.firstinspires.ftc.teamcode.drivetrain;

@Deprecated
public class Coord {
    private double x;
    private double y;
    private double [] coords = {x,y};
    public Coord (double x, double y) {
        this.x = x;
        this.y = y;
        coords[0] = x;
        coords[1] = y;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double [] getCoords() {
        return coords;
    }
    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public void setCoords(double x, double y) {
        this.x = x;
        this.y = y;
        coords[0] = x;
        coords[1] = y;
    }
    public double [] relativeCoords(Coord a) {
        double [] hehe = {x-a.getX(),y-a.getY()};
        return hehe;
    }
}
