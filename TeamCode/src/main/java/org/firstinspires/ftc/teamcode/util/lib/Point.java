package org.firstinspires.ftc.teamcode.util.lib;

public class Point {
    public double x;
    public double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void add(Point point) {
        x += point.x;
        y += point.y;
    }
}
