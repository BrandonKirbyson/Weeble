package org.firstinspires.ftc.teamcode.util.head;

public class HeadOrientation {
    /**
     * The x axis rotation in degrees, controls looking up and down
     */
    public double x;
    /**
     * The y axis rotation in degrees, controls looking left and right
     */
    public double y;
    /**
     * The eyes position, similar to y axis rotation but only affects the eyes
     */
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

