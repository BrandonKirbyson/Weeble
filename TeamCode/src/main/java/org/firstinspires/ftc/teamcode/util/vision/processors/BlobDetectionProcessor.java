package org.firstinspires.ftc.teamcode.util.vision.processors;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.vision.CameraConstants;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class BlobDetectionProcessor implements VisionProcessor {
    public Scalar lower = new Scalar(90, 100, 100);
    public Scalar upper = new Scalar(115, 255, 255);

    private Point center = new Point();

    public Point getCenter() {
        return center;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Not useful in this case, but we do need to implement it either way
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat colMask = preprocessFrame(frame);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(colMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {
            Imgproc.drawContours(frame, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);

            Moments moments = Imgproc.moments(largestContour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();
            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(frame, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
            Imgproc.circle(frame, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            center.x = cX - (double) CameraConstants.WIDTH / 2;
            center.y = cY - (double) CameraConstants.HEIGHT / 2;
        } else {
            center.x = (double) CameraConstants.WIDTH / 2;
            center.y = (double) CameraConstants.HEIGHT / 2;
        }

        return null;
    }

    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Mat colMask = new Mat();
        Core.inRange(hsvFrame, lower, upper, colMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Imgproc.morphologyEx(colMask, colMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(colMask, colMask, Imgproc.MORPH_CLOSE, kernel);

        return colMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
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

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}