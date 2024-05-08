package org.firstinspires.ftc.teamcode.util.head;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.head.HeadConstants;

public class Head {
    private final Servo neckServo;
    private final Servo headServo;
    private final Servo eyesServo;

    private YawPitchRollAngles angles;

    private double eyesTarget = 0;
    private double headTarget = 0;
    private double neckTarget = 0;

    private boolean setPosition = false;

    public Head(HardwareMap hardwareMap) {
        neckServo = hardwareMap.get(Servo.class, "neck");
        headServo = hardwareMap.get(Servo.class, "head");
        eyesServo = hardwareMap.get(Servo.class, "eyes");
    }

    public void updateAngles(YawPitchRollAngles angles) {
        this.angles = angles;
    }

    public void adjustSetPosition(boolean up, boolean down, boolean left, boolean right) {
        setPosition = true;
        double neck = neckServo.getPosition();
        if (up) {
            neck += HeadConstants.neckSpeed;
        } else if (down) {
            neck -= HeadConstants.neckSpeed;
        }
        double head = headServo.getPosition();
        if (left) {
            head += HeadConstants.headSpeed;
        } else if (right) {
            head -= HeadConstants.headSpeed;
        }
        neckServo.setPosition(neck);
        headServo.setPosition(head);

        headTarget = angles.getYaw(AngleUnit.DEGREES) + head * HeadConstants.headConversion;
        neckTarget = angles.getPitch(AngleUnit.DEGREES) + neck * HeadConstants.neckConversion;
    }

    public void setEyes(double manualEyes) {
//        double eyes = eyesServo.getPosition() + manualEyes * HeadConstants.eyesSpeed;

//        eyes = Math.min(HeadConstants.eyesCenter + HeadConstants.eyesRange, Math.max(HeadConstants.eyesCenter - HeadConstants.eyesRange, eyes));
        double eyes = HeadConstants.eyesCenter + (manualEyes * HeadConstants.eyesRange);
        eyesServo.setPosition(eyes);
    }

    public void resetEyes() {
        eyesServo.setPosition(HeadConstants.eyesCenter);
    }

    public void setManualPosition(double manualNeck, double manualHead) {
        setPosition = false;
        double neck = HeadConstants.neckCenter + (-manualNeck * (manualNeck > 0 ? HeadConstants.neckRange : HeadConstants.neckRangeOther));
        double head = HeadConstants.headCenter + (manualHead * HeadConstants.headRange);
        neckServo.setPosition(neck);
        headServo.setPosition(head);

        eyesTarget = angles.getYaw(AngleUnit.DEGREES);
        headTarget = angles.getRoll(AngleUnit.DEGREES);
        neckTarget = angles.getPitch(AngleUnit.DEGREES);
    }

    public void idle() {
        if (setPosition) return;
        double neck = HeadConstants.neckCenter + (angles.getPitch(AngleUnit.DEGREES) / HeadConstants.neckConversion);
        neck = Math.min(HeadConstants.neckCenter + HeadConstants.neckRange, Math.max(HeadConstants.neckCenter - HeadConstants.neckRangeOther, neck));
        if (neckServo.getPosition() != neck) neckServo.setPosition(neck);
    }

    public void reset() {
        eyesServo.setPosition(HeadConstants.eyesCenter);
        neckServo.setPosition(HeadConstants.neckCenter);
        headServo.setPosition(HeadConstants.headCenter);
    }

    public void holdPosition() {
        double head = HeadConstants.headCenter - ((angles.getYaw(AngleUnit.DEGREES) - headTarget) / HeadConstants.headConversion);
        headServo.setPosition(head);
        if (Math.abs(head) > 1) {
            double eyes = HeadConstants.eyesCenter + ((angles.getYaw(AngleUnit.DEGREES) - eyesTarget) / HeadConstants.eyesConversion);
            eyesServo.setPosition(eyes);
        }
        double neck = HeadConstants.neckCenter + ((angles.getPitch(AngleUnit.DEGREES) - neckTarget) / HeadConstants.neckConversion);
        neck = Math.min(HeadConstants.neckCenter + HeadConstants.neckRange, Math.max(HeadConstants.neckCenter - HeadConstants.neckRange, neck));
        neckServo.setPosition(neck);
    }
}
