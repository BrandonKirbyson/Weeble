package org.firstinspires.ftc.teamcode.util.head;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Head {
    private final Servo neckServo;
    private final Servo headServo;
    private final Servo eyesServo;

    private YawPitchRollAngles angles;

    public Head(HardwareMap hardwareMap) {
        neckServo = hardwareMap.get(Servo.class, "neck");
        headServo = hardwareMap.get(Servo.class, "head");
        eyesServo = hardwareMap.get(Servo.class, "eyes");
    }

    public void updateAngles(YawPitchRollAngles angles) {
        this.angles = angles;
    }

    public void setHeadPosition(HeadPosition position) {
        neckServo.setPosition(position.z);
        headServo.setPosition(position.y);
        eyesServo.setPosition(position.eyes);
    }

    private void relativeToWorld(HeadPosition position) {
        position.z = angles.getYaw(AngleUnit.DEGREES) + position.z;
        position.y = angles.getPitch(AngleUnit.DEGREES) + position.y;
        position.eyes = angles.getYaw(AngleUnit.DEGREES) + position.eyes;
    }

    public void worldToRelative(HeadPosition position) {
        position.z = position.z - angles.getYaw(AngleUnit.DEGREES);
        position.y = position.y - angles.getPitch(AngleUnit.DEGREES);
        position.eyes = position.eyes - angles.getYaw(AngleUnit.DEGREES);
    }

}
