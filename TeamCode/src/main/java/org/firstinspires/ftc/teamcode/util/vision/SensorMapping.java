package org.firstinspires.ftc.teamcode.util.vision;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorMapping {
    private final DistanceSensor frontSensor;
    private final DistanceSensor leftSensor;
    private final DistanceSensor rightSensor;

    private double frontDist;
    private double leftDist;
    private double rightDist;

    public SensorMapping(HardwareMap hardwareMap) {
        frontSensor = hardwareMap.get(DistanceSensor.class, "sensor0");
        leftSensor = hardwareMap.get(DistanceSensor.class, "sensor1");
        rightSensor = hardwareMap.get(DistanceSensor.class, "sensor2");
    }

    public void readSensors() {
        frontDist = frontSensor.getDistance(DistanceUnit.INCH);
        leftDist = leftSensor.getDistance(DistanceUnit.INCH);
        rightDist = rightSensor.getDistance(DistanceUnit.INCH);
    }
}
