package org.firstinspires.ftc.teamcode.util.vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.vision.processors.BlobDetectionProcessor;
import org.firstinspires.ftc.teamcode.util.vision.processors.FaceDetectionProcessor;
import org.firstinspires.ftc.teamcode.util.vision.sensors.SensorManager;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

public class Vision {
    private final SensorManager sensorMapping;

    private final VisionPortal visionPortal;

    private VisionMode mode = VisionMode.DISABLED;

    private final BlobDetectionProcessor blobDetectionProcessor;
    private final FaceDetectionProcessor faceDetectionProcessor;

    public Vision(HardwareMap hardwareMap) {
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        sensorMapping = new SensorManager(hardwareMap);

        blobDetectionProcessor = new BlobDetectionProcessor();
        faceDetectionProcessor = new FaceDetectionProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(CameraConstants.WIDTH, CameraConstants.HEIGHT))
                .addProcessors(blobDetectionProcessor, faceDetectionProcessor)
                .build();

        setMode(mode);
    }

    public void setMode(VisionMode mode) {
        this.mode = mode;
        visionPortal.setProcessorEnabled(blobDetectionProcessor, false);
        visionPortal.setProcessorEnabled(faceDetectionProcessor, false);
        switch (mode) {
//            case DISABLED:
//                if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
//                    visionPortal.stopStreaming();
//                }
//                break;
            case BLOB_TRACKING:
                visionPortal.setProcessorEnabled(blobDetectionProcessor, true);
                break;
            case FACE_TRACKING:
                visionPortal.setProcessorEnabled(faceDetectionProcessor, true);
                break;
        }
    }

    public Point getTrackingCenter() {
        switch (mode) {
            case BLOB_TRACKING:
                return blobDetectionProcessor.getCenter();
            case FACE_TRACKING:
                return faceDetectionProcessor.getCenter();
            default:
                return null;
        }
    }

    public void update() {
//        sensorMapping.update();
    }
}
