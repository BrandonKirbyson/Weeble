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

    public void reset() {
        HeadOrientation position = new HeadOrientation(0, 0, 0);
        setHeadPosition(position);
    }

    public void manualControl(double x, double y) {
        HeadOrientation position = new HeadOrientation(x * 90, y * 90);
        setHeadPosition(position);
    }

    public void setHeadPosition(HeadOrientation position) {
        neckServo.setPosition(position.x / HeadConstants.xConversion + HeadConstants.xCenter);
        headServo.setPosition(position.y / HeadConstants.yConversion + HeadConstants.yCenter);
        eyesServo.setPosition(position.eyes);
    }

    private void relativeToWorld(HeadOrientation position) {
        position.x = position.x - angles.getPitch(AngleUnit.DEGREES);
        position.y = position.y - angles.getRoll(AngleUnit.DEGREES);
        position.eyes = position.eyes + angles.getYaw(AngleUnit.DEGREES);
    }

    public void worldToRelative(HeadOrientation position) {
        position.x = position.x + angles.getPitch(AngleUnit.DEGREES);
        position.y = position.y + angles.getRoll(AngleUnit.DEGREES);
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

        if (animationIndex >= currentAnimation.size()) {
            currentAnimation = null;
            return;
        }

        HeadAnimationKeyframe key = currentAnimation.get(animationIndex);
        if (animationTimer.seconds() >= key.time) {
            setHeadPosition(key.position);
            animationIndex++;
            animationTimer.reset();
        } else {
            double progress = animationTimer.seconds() / (double) key.time;
            progress = HeadAnimationKeyframe.getTransitionProgress(progress, key.transitionType);
            HeadAnimationKeyframe lastKey = currentAnimation.get(animationIndex - 1);
            HeadOrientation position = new HeadOrientation(
                    key.position.y + ((key.position.y - lastKey.position.y) * progress),
                    key.position.x + ((key.position.x - lastKey.position.x) * progress),
                    key.position.eyes + ((key.position.eyes - lastKey.position.eyes) * progress)
            );
            setHeadPosition(position);
        }
    }
}
