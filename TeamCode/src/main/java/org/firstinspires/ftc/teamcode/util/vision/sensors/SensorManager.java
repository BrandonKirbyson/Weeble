package org.firstinspires.ftc.teamcode.util.vision.sensors;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.lib.FtcDashboardManager;
import org.firstinspires.ftc.teamcode.util.lib.Point;
import org.firstinspires.ftc.teamcode.util.lib.Pose;

public class SensorManager {
    private final DistanceSensor frontSensor;
    private final DistanceSensor leftSensor;
    private final DistanceSensor rightSensor;

    private double frontDist;
    private double leftDist;
    private double rightDist;

    private final MappingManager mapping = new MappingManager();

    public SensorManager(HardwareMap hardwareMap) {
        frontSensor = hardwareMap.get(DistanceSensor.class, "sensor0");
        leftSensor = hardwareMap.get(DistanceSensor.class, "sensor1");
        rightSensor = hardwareMap.get(DistanceSensor.class, "sensor2");
    }

    public void readSensors() {
        frontDist = frontSensor.getDistance(DistanceUnit.INCH);
        leftDist = leftSensor.getDistance(DistanceUnit.INCH);
        rightDist = rightSensor.getDistance(DistanceUnit.INCH);
    }

    public void update() {
        readSensors();

        mapping.processSensors(frontDist, leftDist, rightDist, new Pose(0, 0, 0));

        overlayMapping();
    }

    private void overlayMapping() {
        Canvas canvas = FtcDashboardManager.getPacket().field();

        for (Point point : mapping.getPoints()) {
            canvas.strokeCircle(point.x, point.y, 2);
        }
    }
}
