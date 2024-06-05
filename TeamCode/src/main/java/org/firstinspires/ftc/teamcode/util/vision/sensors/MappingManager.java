package org.firstinspires.ftc.teamcode.util.vision.sensors;

import org.firstinspires.ftc.teamcode.util.lib.Point;
import org.firstinspires.ftc.teamcode.util.lib.Pose;

import java.util.ArrayList;

public class MappingManager {
    private final ArrayList<Point> points = new ArrayList<>();

    public void processSensors(double frontDist, double leftDist, double rightDist, Pose pose) {
        Point front = polarToCartesian(pose.heading, frontDist + SensorConstants.FrontZOffset);
        Point left = polarToCartesian(pose.heading + Math.PI / 2, leftDist + SensorConstants.LeftXOffset);
        Point right = polarToCartesian(pose.heading - Math.PI / 2, rightDist + SensorConstants.RightXOffset);

        Point robot = new Point(pose.x, pose.y);
        front.add(robot);
        left.add(robot);
        right.add(robot);

        points.add(front);
        points.add(left);
        points.add(right);
    }

    private Point polarToCartesian(double angle, double distance) {
        return new Point(distance * Math.cos(angle), distance * Math.sin(angle));
    }

    public ArrayList<Point> getPoints() {
        return points;
    }
}
