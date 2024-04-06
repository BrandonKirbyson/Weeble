package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.constants.EyeConstants;

public class Eyes {
    private final Servo tiltServo;
    private final Servo panServo;

    public Eyes(HardwareMap hardwareMap) {
        tiltServo = hardwareMap.get(Servo.class, "eyesTilt");
        panServo = hardwareMap.get(Servo.class, "eyesPan");
    }

    public void update(double manualPan, double manualTilt) {
        panServo.setPosition(EyeConstants.panCenter + (manualPan * EyeConstants.panRange));
        tiltServo.setPosition(EyeConstants.tiltCenter + (manualTilt * EyeConstants.tiltRange));
    }
}
