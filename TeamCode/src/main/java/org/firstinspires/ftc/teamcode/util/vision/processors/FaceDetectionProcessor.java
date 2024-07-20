package org.firstinspires.ftc.teamcode.util.vision.processors;

import android.graphics.Canvas;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

@Config
public class FaceDetectionProcessor implements VisionProcessor {
    public static Scalar lowerFace = new Scalar(100, 25, 50);
    public static Scalar upperFace = new Scalar(126, 185, 255);

    public static Scalar lowerEye = new Scalar(136, 0, 0);
    public static Scalar upperEye = new Scalar(255, 255, 120);

    public static double blur = 1;

    public static double faceSolidity = 0.4;
    public static double faceArea = 500;
    public static double minPerimeter = 100;

    public static double minBlobSize = 5;
    public static double minEyes = 2;

    private Point center = null;

    private boolean findFace = true;

    public static boolean mask = false;

    public Point getCenter() {
        return center;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    public void findFace() {
        center = null;
        findFace = true;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // remove background from frame using colMask
        Mat colMask = preprocessFrame(frame);

        // Find contours of the detected yellow regions
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(colMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contours = filterShape(contours);

        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Mat eyesMask = new Mat();
        Core.inRange(hsvFrame, lowerEye, upperEye, eyesMask);
        int s = 2;
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(s, s));
        Imgproc.morphologyEx(eyesMask, eyesMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(eyesMask, eyesMask, Imgproc.MORPH_CLOSE, kernel);

//        eyesMask.copyTo(frame);

        List<MatOfPoint> eyes = new ArrayList<>();
        Mat hierarchyEyes = new Mat();
        Imgproc.findContours(eyesMask, eyes, hierarchyEyes, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Mat frameNoBackground = new Mat();
        Core.bitwise_and(frame, frame, frameNoBackground, colMask);

//        frameNoBackground.copyTo(frame);

        List<MatOfPoint> filteredEyes = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            Rect boundingBox = Imgproc.boundingRect(contour);
            int c = 0;
            for (MatOfPoint eye : eyes) {
                Rect eyeBoundingBox = Imgproc.boundingRect(eye);
                if (eyeBoundingBox.x > boundingBox.x && eyeBoundingBox.y > boundingBox.y && eyeBoundingBox.x + eyeBoundingBox.width < boundingBox.x + boundingBox.width && eyeBoundingBox.y + eyeBoundingBox.height < boundingBox.y + boundingBox.height) {
                    c++;
                    filteredEyes.add(eye);
                }
            }
            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();


            if (c < minEyes) {
                contours.remove(i);
                i--;
            }
        }

//        Imgproc.drawContours(frame, contours, -1, new Scalar(0, 255, 0), 2);

//        Imgproc.drawContours(frame, filteredEyes, -1, new Scalar(0, 0, 255), 1);

        for (MatOfPoint contour : contours) {
            Rect boundingBox = Imgproc.boundingRect(contour);
            Imgproc.rectangle(frame, boundingBox.tl(), boundingBox.br(), new Scalar(255, 0, 0), 2);
        }
        MatOfPoint largestContour = VisionProcessorUtil.findLargestContour(contours);

        if (largestContour != null) {
            Moments moments = Imgproc.moments(largestContour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();
            center = new Point(cX - (double) frame.width() / 2, cY - (double) frame.height() / 2);

            Rect boundingBox = Imgproc.boundingRect(largestContour);
            Imgproc.rectangle(frame, boundingBox.tl(), boundingBox.br(), new Scalar(0, 255, 0), 3);
        } else {
            center = null;
        }

        if (mask) {
            Imgproc.cvtColor(colMask, colMask, Imgproc.COLOR_GRAY2RGBA);
            colMask.copyTo(frame);
        }

        return null;
    }

    private Mat preprocessFrame(Mat frame) {
        if (blur > 0) {
            Imgproc.GaussianBlur(frame, frame, new Size(0, 0), blur);
        }
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Mat colMask = new Mat();
        Core.inRange(hsvFrame, lowerFace, upperFace, colMask);

        hsvFrame.release();

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(minBlobSize, minBlobSize));
        Imgproc.morphologyEx(colMask, colMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(colMask, colMask, Imgproc.MORPH_CLOSE, kernel);

        return colMask;
    }

    private List<MatOfPoint> filterShape(List<MatOfPoint> contours) {
        List<MatOfPoint> filteredContours = new ArrayList<>();

        for (MatOfPoint contour : contours) {
            Rect boundingBox = Imgproc.boundingRect(contour);

            double aspectRatio = (double) boundingBox.width / boundingBox.height;
            double area = Imgproc.contourArea(contour);
            double perimeter = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
            double solidity = area / (boundingBox.width * boundingBox.height);

            if (aspectRatio > 0.5 && aspectRatio < 2 && area > faceArea && solidity > faceSolidity && perimeter > minPerimeter) {
                filteredContours.add(contour);
            }
        }

        return filteredContours;
    }

    private List<MatOfPoint> filterEyes(List<MatOfPoint> contours, Mat frame) {
        List<MatOfPoint> filteredContours = new ArrayList<>();

        for (MatOfPoint contour : contours) {
            //get bounding box of contour and crop it
            Rect boundingBox = Imgproc.boundingRect(contour);
            Moments moments = Imgproc.moments(contour);
            boundingBox.x = (int) (moments.get_m10() / moments.get_m00());
            boundingBox.y = (int) (moments.get_m01() / moments.get_m00());
            Mat face = frame.submat(boundingBox);

            //convert to grayscale
            Mat grayFace = new Mat();
            Imgproc.cvtColor(face, grayFace, Imgproc.COLOR_BGR2GRAY);

            //detect eyes
            List<MatOfPoint> eyes = new ArrayList<>();
            Imgproc.findContours(grayFace, eyes, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            //filter eyes
            filteredContours.add(contour);
            filteredContours.addAll(eyes);
        }

        return filteredContours;
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