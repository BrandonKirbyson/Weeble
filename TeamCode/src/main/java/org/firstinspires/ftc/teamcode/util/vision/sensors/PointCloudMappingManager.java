package org.firstinspires.ftc.teamcode.util.vision.sensors;

import org.firstinspires.ftc.teamcode.util.drive.Pose;

import java.util.ArrayList;

public class PointCloudMappingManager {
    private final ArrayList<MapPoint> points = new ArrayList<>();

    public void processSensors(double frontDist, double leftDist, double rightDist, Pose pose) {
        MapPoint front = polarToCartesian(pose.heading, frontDist + SensorConstants.FrontZOffset);
        MapPoint left = polarToCartesian(pose.heading + Math.PI / 2, leftDist + SensorConstants.LeftXOffset);
        MapPoint right = polarToCartesian(pose.heading - Math.PI / 2, rightDist + SensorConstants.RightXOffset);

        MapPoint robot = new MapPoint(pose.x, pose.y);
        front.add(robot);
        left.add(robot);
        right.add(robot);

        points.add(front);
        points.add(left);
        points.add(right);
    }

    private MapPoint polarToCartesian(double angle, double distance) {
        return new MapPoint(distance * Math.cos(angle), distance * Math.sin(angle));
    }

    public ArrayList<MapPoint> getPoints() {
        return points;
    }
}
