package org.firstinspires.ftc.teamcode.util.vision.processors;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class FaceDetectionProcessor implements VisionProcessor {
    public Scalar lowerFace = new Scalar(100, 25, 50);
    public Scalar upperFace = new Scalar(126, 185, 255);

    public Scalar lowerEye = new Scalar(136, 0, 0);
    public Scalar upperEye = new Scalar(255, 255, 120);

    public double blur = 1;

    public double faceSolidity = 0.4;
    public double faceArea = 500;
    public double minPerimeter = 100;

    public double minBlobSize = 5;
    public double minEyes = 2;

    private Point center = null;

    private boolean findFace = false;

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
        Mat colMask = preprocessFrame(frame);

        // remove background from frame using colMask


//        colMask.copyTo(frame);

        // Find contours of the detected yellow regions
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(colMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contours = filterShape(contours);

//        colMask.release();

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

//        colMask.copyTo(frame);


//        List<MatOfPoint> filteredContours = new ArrayList<>();
//        for (MatOfPoint eye : eyes) {
//            Rect boundingBox = Imgproc.boundingRect(eye);
//            for (int i = 0; i < contours.size(); i++) {
//            for (MatOfPoint contour : contours) {
//                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
//                if (Imgproc.pointPolygonTest(contour2f, new Point(boundingBox.x, boundingBox.y), false) > 0) {
//                    filteredContours.add(eye);
//                }
//            }
//            }
//        }


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
            } else {
//                Imgproc.putText(frame, c + "", new Point(cX, cY + (double) boundingBox.height / 2 + 30), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(100, 200, 255), 2);
            }
        }


//        Imgproc.drawContours(frame, contours, -1, new Scalar(0, 255, 0), 2);

//        Imgproc.drawContours(frame, filteredEyes, -1, new Scalar(0, 0, 255), 1);

        for (MatOfPoint contour : contours) {
            Rect boundingBox = Imgproc.boundingRect(contour);
            Imgproc.rectangle(frame, boundingBox.tl(), boundingBox.br(), new Scalar(255, 0, 0), 2);

            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();
//            Imgproc.circle(frame, new Point(cX, cY), 1, new Scalar(255, 0, 0), 2);
        }


//
//        // Find the largest yellow contour (blob)
//        MatOfPoint largestContour = findLargestContour(contours);
//
//        if (largestContour != null) {
//            // Draw a red outline around the largest detected object
//            Imgproc.drawContours(frame, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
//
//            // Calculate the centroid of the largest contour
//            Moments moments = Imgproc.moments(largestContour);
//            double cX = moments.get_m10() / moments.get_m00();
//            double cY = moments.get_m01() / moments.get_m00();
//
//            // Draw a dot at the centroid
//            String label = "(" + (int) cX + ", " + (int) cY + ")";
//            Imgproc.putText(frame, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//            Imgproc.circle(frame, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
//
//        }

//

        if (findFace && center == null) {
            MatOfPoint largestContour = VisionProcessorUtil.findLargestContour(contours);
            Moments moments = Imgproc.moments(largestContour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();
            center.x = cX - (double) frame.width() / 2;
            center.y = cY - (double) frame.height() / 2;
        } else {
            center = null;
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