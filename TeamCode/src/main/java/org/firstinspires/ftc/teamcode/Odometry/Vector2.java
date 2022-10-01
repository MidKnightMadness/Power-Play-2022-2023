package org.firstinspires.ftc.teamcode.Odometry;

// todo get string repr

public class Vector2 {
    double x;
    double y;

    public Vector2 (double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2() {
        this.x = 0;
        this.y = 0;
    }

    public String toString() {
        return String.format("(%f, %f)", this.x, this.y);
    }
}
