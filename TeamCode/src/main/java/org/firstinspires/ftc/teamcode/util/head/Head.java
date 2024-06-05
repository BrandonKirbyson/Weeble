package org.firstinspires.ftc.teamcode.util.head;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;

public class Head {
    private final Servo neckServo;
    private final Servo headServo;
    private final Servo eyesServo;

    private YawPitchRollAngles angles;

    private ArrayList<HeadAnimationKeyframe> currentAnimation = null;
    private final ElapsedTime animationTimer = new ElapsedTime();
    private int animationIndex = 0;

    public Head(HardwareMap hardwareMap) {
        neckServo = hardwareMap.get(Servo.class, "neck");
        headServo = hardwareMap.get(Servo.class, "head");
        eyesServo = hardwareMap.get(Servo.class, "eyes");
    }

    public void updateAngles(YawPitchRollAngles angles) {
        this.angles = angles;
    }

    public void update() {
        if (currentAnimation != null) {
            updateAnimation();
        }
    }

    public void manualControl(double y, double z) {
        HeadPosition position = new HeadPosition(y, z);
        setHeadPosition(position);
    }

    public void setHeadPosition(HeadPosition position) {
        neckServo.setPosition(position.z);
        headServo.setPosition(position.y);
        eyesServo.setPosition(position.eyes);
    }

    private void relativeToWorld(HeadPosition position) {
        position.z = HeadConstants.zCenter + ((angles.getYaw(AngleUnit.DEGREES) - position.z) / HeadConstants.zConversion);
        position.y = HeadConstants.yCenter + ((angles.getYaw(AngleUnit.DEGREES) - position.y) / HeadConstants.yConversion);
        position.eyes = angles.getYaw(AngleUnit.DEGREES) + position.eyes;
    }

    public void worldToRelative(HeadPosition position) {
        position.z = angles.getYaw(AngleUnit.DEGREES) - ((position.z - HeadConstants.zCenter) * HeadConstants.zConversion);
        position.y = angles.getYaw(AngleUnit.DEGREES) - ((position.y - HeadConstants.yCenter) * HeadConstants.yConversion);
        position.eyes = position.eyes - angles.getYaw(AngleUnit.DEGREES);
    }

    public void runAnimation(ArrayList<HeadAnimationKeyframe> animation) {
        currentAnimation = animation;
        animationIndex = 0;
        animationTimer.reset();
    }

    private void updateAnimation() {
        if (currentAnimation.isEmpty()) {
            currentAnimation = null;
            return;
        }

        HeadAnimationKeyframe key = currentAnimation.get(animationIndex);
        if (animationTimer.milliseconds() >= key.time) {
            setHeadPosition(key.position);
            animationIndex++;
            animationTimer.reset();
        } else {
            double progress = animationTimer.milliseconds() / (double) key.time;
            progress = HeadAnimationKeyframe.getTransitionProgress(progress, key.transitionType);
            HeadAnimationKeyframe lastKey = currentAnimation.get(animationIndex - 1);
            HeadPosition position = new HeadPosition(
                    key.position.y + ((key.position.y - lastKey.position.y) * progress),
                    key.position.z + ((key.position.z - lastKey.position.z) * progress),
                    key.position.eyes + ((key.position.eyes - lastKey.position.eyes) * progress)
            );
            worldToRelative(position);
            setHeadPosition(position);
        }
    }
}
