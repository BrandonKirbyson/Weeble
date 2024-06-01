package org.firstinspires.ftc.teamcode.util.head;

public class HeadPosition {
    public double z;
    public double y;
    public double eyes;

    public HeadPosition(double z, double y) {
        this.z = z;
        this.y = y;
        this.eyes = z;
    }

    public HeadPosition(double z, double y, double eyes) {
        this.z = z;
        this.y = y;
        this.eyes = eyes;
    }
}
