package org.firstinspires.ftc.teamcode.util.vision.processors;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.List;

public class VisionProcessorUtil {
    public static MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }

    public static MatOfPoint findClosestContour(List<MatOfPoint> contours, Point target) {
        double minDistance = Double.MAX_VALUE;
        MatOfPoint closestContour = null;

        for (MatOfPoint contour : contours) {
            Moments moments = Imgproc.moments(contour);
            Point center = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

            double distance = Math.hypot(center.x - target.x, center.y - target.y);
            if (distance < minDistance) {
                minDistance = distance;
                closestContour = contour;
            }
        }

        return closestContour;
    }
}
