package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.constants.HeadConstants;

public class Head {
    private final Servo neckServo;
    private final Servo headServo;
    private final Servo eyesServo;

    private final IMU imu;

    private double eyesTargetHeading = 0;

    public Head(HardwareMap hardwareMap, IMU imu) {
        neckServo = hardwareMap.get(Servo.class, "neck");
        headServo = hardwareMap.get(Servo.class, "head");
        eyesServo = hardwareMap.get(Servo.class, "eyes");
        this.imu = imu;
    }

    public void setManualPosition(double manualEyes, double manualNeck, double manualHead) {
        double eyes = HeadConstants.eyesCenter + (manualEyes * HeadConstants.eyesRange);
        double neck = HeadConstants.neckCenter + (manualNeck * HeadConstants.neckRange);
        double head = HeadConstants.headCenter + (manualHead * HeadConstants.headRange);
        eyesServo.setPosition(eyes);
        neckServo.setPosition(neck);
        headServo.setPosition(head);

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        eyesTargetHeading = angles.getYaw(AngleUnit.DEGREES);
    }

    public void holdPosition() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double eyes = HeadConstants.eyesCenter + ((angles.getYaw(AngleUnit.DEGREES) - eyesTargetHeading) / HeadConstants.eyesConversion);
        eyesServo.setPosition(eyes);
    }
}
