package org.firstinspires.ftc.teamcode.ITD;

public class Vector2 {
    public double x;
    public double y;
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public double getDist(Vector2 v) {
        return Math.sqrt((v.x - x) * (v.x - x) + (v.y - y) * (v.y - y));
    }
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }
    public Vector2 normalize() {
        double mag = magnitude();
        return new Vector2(x / mag, y / mag);
    }
    public Vector2 add(Vector2 v) {
        return new Vector2(x + v.x, y + v.y);
    }
    public Vector2 subtract(Vector2 v) {
        return new Vector2(x - v.x, y - v.y);
    }
    public Vector2 multiply(double scalar) {
        return new Vector2(x * scalar, y * scalar);
    }
    public boolean equals(Vector2 v) {
        return x == v.x && y == v.y;
    }
    public boolean equals(Vector2 v, double tolerance) {
        return Math.abs(x - v.x) < tolerance && Math.abs(y - v.y) < tolerance;
    }


    public Vector2 rotate(double angle) {
        double x = this.x * Math.cos(angle) - this.y * Math.sin(angle);
        double y = this.x * Math.sin(angle) + this.y * Math.cos(angle);
        return new Vector2(x, y);
    }

    public String toString() {
        return "(" + x + " , " + y + ")";
    }
}
