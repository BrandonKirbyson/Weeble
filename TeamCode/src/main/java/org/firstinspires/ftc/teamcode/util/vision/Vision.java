package org.firstinspires.ftc.teamcode.util.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.vision.sensors.SensorManager;

public class Vision {
    private final WebcamName webcam;
    private final SensorManager sensorMapping;

    public Vision(HardwareMap hardwareMap) {
        this.webcam = hardwareMap.get(WebcamName.class, "webcam");
        sensorMapping = new SensorManager(hardwareMap);
    }

    public void update() {
        sensorMapping.update();
    }
}
