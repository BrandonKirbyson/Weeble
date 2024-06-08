package org.firstinspires.ftc.teamcode.util.arms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arms {
    private final Servo leftArm;
    private final Servo rightArm;

    public Arms(HardwareMap hardwareMap) {
        leftArm = hardwareMap.get(Servo.class, "arm1");
        rightArm = hardwareMap.get(Servo.class, "arm0");
        leftArm.setDirection(Servo.Direction.REVERSE);
    }

    public void setArmPosition(double position) {
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }
}
