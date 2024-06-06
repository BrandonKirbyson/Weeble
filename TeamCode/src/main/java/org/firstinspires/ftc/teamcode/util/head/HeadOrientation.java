package org.firstinspires.ftc.teamcode.util.head;

public class HeadOrientation {
    public double x;
    public double y;
    public double eyes;

    public HeadOrientation(double x, double y, double eyes) {
        this.x = Math.min(HeadConstants.xMax, Math.max(HeadConstants.xMin, x));
        this.y = Math.min(HeadConstants.yMax, Math.max(HeadConstants.yMin, y));
        this.eyes = Math.min(HeadConstants.eyesMax, Math.max(HeadConstants.eyesMin, eyes));
    }

    public HeadOrientation(double x, double y) {
        this(x, y, 0);
    }
}

