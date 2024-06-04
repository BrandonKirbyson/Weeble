package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "SensorTest", group = "tests")
public class SensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DistanceSensor front = hardwareMap.get(DistanceSensor.class, "sensor0");
        DistanceSensor left = hardwareMap.get(DistanceSensor.class, "sensor1");
        DistanceSensor right = hardwareMap.get(DistanceSensor.class, "sensor2");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Front Dist", front.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left Dist", left.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Dist", right.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}