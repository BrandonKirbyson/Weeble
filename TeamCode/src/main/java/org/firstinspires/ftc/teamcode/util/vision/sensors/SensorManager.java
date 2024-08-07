package org.firstinspires.ftc.teamcode.util.vision.sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.drive.Pose;

import java.util.ArrayList;

public class SensorManager {
    private final DistanceSensor frontSensor;
    private final DistanceSensor leftSensor;
    private final DistanceSensor rightSensor;

    private double frontDist;
    private double leftDist;
    private double rightDist;

    private final PointCloudMappingManager mapping = new PointCloudMappingManager();

    public SensorManager(HardwareMap hardwareMap) {
        frontSensor = hardwareMap.get(DistanceSensor.class, "sensor2");
        leftSensor = hardwareMap.get(DistanceSensor.class, "sensor3");
        rightSensor = hardwareMap.get(DistanceSensor.class, "sensor1");
    }

    public void readSensors() {
        frontDist = frontSensor.getDistance(DistanceUnit.INCH);
        leftDist = leftSensor.getDistance(DistanceUnit.INCH);
        rightDist = rightSensor.getDistance(DistanceUnit.INCH);
    }

    public void update() {
        readSensors();

        mapping.processSensors(frontDist, leftDist, rightDist, new Pose(0, 0, 0));
    }

    public ArrayList<MapPoint> getPoints() {
        return mapping.getPoints();
    }
}
