package org.firstinspires.ftc.teamcode.util.overlay;

import com.acmerobotics.dashboard.canvas.Canvas;
import org.firstinspires.ftc.teamcode.util.drive.Pose;
import org.firstinspires.ftc.teamcode.util.lib.FtcDashboardManager;
import org.firstinspires.ftc.teamcode.util.vision.sensors.MapPoint;

import java.util.ArrayList;

public class OverlayManager {
    private final ArrayList<MapPoint> points = new ArrayList<>();
    private final ArrayList<MapPoint> posePath = new ArrayList<>();
    private Pose pose = new Pose();

    private Canvas canvas = FtcDashboardManager.getPacket().fieldOverlay();

    private Canvas getCanvas() {
        return FtcDashboardManager.getPacket().fieldOverlay();
    }

    public void update() {
        canvas = FtcDashboardManager.getPacket().fieldOverlay();

        getCanvas()
                .setFill(OverlayConstants.background)
                .setScale(OverlayConstants.scale, OverlayConstants.scale);

        posePath.add(new MapPoint(pose.x, pose.y));
        if (posePath.size() > OverlayConstants.robotPathLength) {
            posePath.remove(0);
        }

        drawRobot();
        drawPoints();
    }

    public void updatePoints(ArrayList<MapPoint> points) {
        this.points.clear();
        this.points.addAll(points);
    }

    public void updatePose(Pose pose) {
        this.pose = pose;
    }

    private void drawRobot() {
        canvas.setFill("transparent");
        double x = pose.x;
        double y = pose.y;
        double heading = pose.heading;
        double w = OverlayConstants.robotWidth;
        double l = OverlayConstants.robotLength;
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        // Rectangle representing the robot facing the proper heading
        canvas
                .setStroke(OverlayConstants.robotColor)
                .fillPolygon(new double[]{
                        x + l / 2 * cos - w / 2 * sin,
                        x + l / 2 * cos + w / 2 * sin,
                        x - l / 2 * cos + w / 2 * sin,
                        x - l / 2 * cos - w / 2 * sin
                }, new double[]{
                        y + l / 2 * sin + w / 2 * cos,
                        y + l / 2 * sin - w / 2 * cos,
                        y - l / 2 * sin - w / 2 * cos,
                        y - l / 2 * sin + w / 2 * cos
                });
        // Circle representing the turning path
        canvas
                .setStroke(OverlayConstants.robotHeadingColor)
                .strokeCircle(x, y, OverlayConstants.robotWidth);
        // Line representing the heading
        canvas
                .setStroke(OverlayConstants.robotHeadingColor)
                .strokeLine(x, y, x + l / 2 * cos, y + l / 2 * sin);
        // Curve representing the path of the robot
        canvas
                .setStroke(OverlayConstants.robotPathColor)
                .strokePolyline(posePath.stream().mapToDouble(point -> point.x).toArray(), posePath.stream().mapToDouble(point -> point.y).toArray());
    }

    private void drawPoints() {
        canvas.setFill(OverlayConstants.pointColor);
        for (MapPoint point : points) {
            canvas.strokeCircle(point.x, point.y, OverlayConstants.pointSize);
        }
    }
}
