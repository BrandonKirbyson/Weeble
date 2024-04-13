package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.constants.BalanceConstants;
import org.firstinspires.ftc.teamcode.util.constants.EyeConstants;

public class Eyes {
    private final Servo tiltServo;
    private final Servo panServo;

    private final IMU imu;

    private double targetPan = 0;
    private double targetTilt = 0;

    public Eyes(HardwareMap hardwareMap, IMU imu) {
        tiltServo = hardwareMap.get(Servo.class, "eyesTilt");
        panServo = hardwareMap.get(Servo.class, "eyesPan");
        this.imu = imu;
    }

    public void setManualPosition(double manualPan, double manualTilt) {
        double pan = EyeConstants.panCenter + (manualPan * EyeConstants.panRange);
        double tilt = EyeConstants.tiltCenter + (manualTilt * EyeConstants.tiltRange);
        panServo.setPosition(pan);
        tiltServo.setPosition(tilt);


        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        targetPan = angles.getYaw(AngleUnit.DEGREES);
        targetTilt = tilt;
    }

    public void holdPosition() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double pan = EyeConstants.panCenter + ((angles.getYaw(AngleUnit.DEGREES) - targetPan) / EyeConstants.panConversion);
        double tilt = targetTilt;
        panServo.setPosition(pan);
        tiltServo.setPosition(tilt);
    }
}
