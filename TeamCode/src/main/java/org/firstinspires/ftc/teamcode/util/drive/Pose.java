package org.firstinspires.ftc.teamcode.util.drive;

public class Pose {
    public double x;
    public double y;
    public double heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(Pose pose) {
        this(pose.x, pose.y, pose.heading);
    }
}
