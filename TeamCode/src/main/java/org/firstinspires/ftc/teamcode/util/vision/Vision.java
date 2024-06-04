package org.firstinspires.ftc.teamcode.util.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Vision {
    private final WebcamName webcam;
    private final SensorMapping sensorMapping;

    public Vision(HardwareMap hardwareMap) {
        this.webcam = hardwareMap.get(WebcamName.class, "webcam");
        sensorMapping = new SensorMapping(hardwareMap);
    }

    public void readSensors() {
        sensorMapping.readSensors();
    }
}
