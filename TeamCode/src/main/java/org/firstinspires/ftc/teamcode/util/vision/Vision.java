package org.firstinspires.ftc.teamcode.util.vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.vision.processors.BlobDetectionProcessor;
import org.firstinspires.ftc.teamcode.util.vision.sensors.SensorManager;
import org.firstinspires.ftc.vision.VisionPortal;

public class Vision {
    private final WebcamName webcam;
    private final SensorManager sensorMapping;

    private final VisionPortal visionPortal;

    public Vision(HardwareMap hardwareMap) {
        this.webcam = hardwareMap.get(WebcamName.class, "webcam");
        sensorMapping = new SensorManager(hardwareMap);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(CameraConstants.WIDTH, CameraConstants.HEIGHT))
                .addProcessor(new BlobDetectionProcessor())
                .build();
    }

    public void update() {
        sensorMapping.update();
    }
}
