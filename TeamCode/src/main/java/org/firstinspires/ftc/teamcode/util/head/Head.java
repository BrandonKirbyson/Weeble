package org.firstinspires.ftc.teamcode.util.head;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.drive.PIDController;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Head {
    private final Servo neckServo;
    private final Servo headServo;
    private final Servo eyesServo;

    private YawPitchRollAngles angles;

    private HeadOrientation currentPosition = new HeadOrientation(0, 0, 0);

    private ArrayList<HeadAnimationKeyframe> currentAnimation = null;
    private final ElapsedTime animationTimer = new ElapsedTime();
    private int animationIndex = 0;

    private final PIDController xTrackingPID = new PIDController(HeadConstants.xTrackingPID);
    private final PIDController yTrackingPID = new PIDController(HeadConstants.yTrackingPID);
    private boolean tracking = false;

    private boolean manualControl = false;

    public Head(HardwareMap hardwareMap) {
        neckServo = hardwareMap.get(Servo.class, "neck");
        headServo = hardwareMap.get(Servo.class, "head");
        eyesServo = hardwareMap.get(Servo.class, "eyes");
    }

    public void updateAngles(YawPitchRollAngles angles) {
        this.angles = angles;
    }

    private void updateCurrentHeadOrientation() {
        currentPosition = new HeadOrientation(
                headServo.getPosition(),
                neckServo.getPosition(),
                eyesServo.getPosition()
        );

        servoPositionToOrientation(currentPosition);
    }

    public void update(Point trackingTarget) {
        updateCurrentHeadOrientation();

        if (currentAnimation != null) {
            updateAnimation();
        } else if (tracking && trackingTarget != null) {
            trackObject(trackingTarget);
        }
    }

    public void reset() {
        HeadOrientation position = new HeadOrientation(0, 0, 0);
        setHeadPosition(position);
        tracking = false;
    }

    public void manualControl(double x, double y) {
        HeadOrientation position = new HeadOrientation(x * 90, y * 90);
        setHeadPosition(position);
        tracking = false;
    }

    public void setHeadPosition(HeadOrientation position) {
        neckServo.setPosition(position.x / HeadConstants.xConversion + HeadConstants.xCenter);
        headServo.setPosition(position.y / HeadConstants.yConversion + HeadConstants.yCenter);
        eyesServo.setPosition(position.eyes / HeadConstants.eyesConversion + HeadConstants.eyesCenter);
    }

    private void relativeToWorld(HeadOrientation position) {
        position.x = position.x - angles.getPitch(AngleUnit.DEGREES);
        position.y = position.y - angles.getRoll(AngleUnit.DEGREES);
        position.eyes = position.eyes + angles.getYaw(AngleUnit.DEGREES);
    }

    private void worldToRelative(HeadOrientation position) {
        position.x = position.x + angles.getPitch(AngleUnit.DEGREES);
        position.y = position.y + angles.getRoll(AngleUnit.DEGREES);
        position.eyes = position.eyes - angles.getYaw(AngleUnit.DEGREES);
    }

    private void servoPositionToOrientation(HeadOrientation position) {
        position.x = (neckServo.getPosition() - HeadConstants.xCenter) * HeadConstants.xConversion;
        position.y = (headServo.getPosition() - HeadConstants.yCenter) * HeadConstants.yConversion;
        position.eyes = (eyesServo.getPosition() - HeadConstants.eyesCenter) * HeadConstants.eyesConversion;
    }

    private void orientationToServoPosition(HeadOrientation position) {
        neckServo.setPosition(position.x / HeadConstants.xConversion + HeadConstants.xCenter);
        headServo.setPosition(position.y / HeadConstants.yConversion + HeadConstants.yCenter);
        eyesServo.setPosition(position.eyes / HeadConstants.eyesConversion + HeadConstants.eyesCenter);
    }

    public void runAnimation(ArrayList<HeadAnimationKeyframe> animation) {
        currentAnimation = animation;
        animationIndex = 0;
        animationTimer.reset();
        tracking = false;
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
            HeadOrientation lastPosition = animationIndex > 0 ? currentAnimation.get(animationIndex - 1).position : currentPosition;
            HeadOrientation position = new HeadOrientation(
                    key.position.y + ((key.position.y - lastPosition.y) * progress),
                    key.position.x + ((key.position.x - lastPosition.x) * progress),
                    key.position.eyes + ((key.position.eyes - lastPosition.eyes) * progress)
            );
            setHeadPosition(position);
        }
    }

    public void setTracking(boolean tracking) {
        this.tracking = tracking;
    }

    public void trackObject(Point target) {
        // Flipped x and y because it is 2d point to 3d rotational axis
        double xError = target.y - currentPosition.x;
        double yError = target.x - currentPosition.y;

        double xOutput = currentPosition.x + xTrackingPID.update(xError);
        double yOutput = currentPosition.y + yTrackingPID.update(yError);

        HeadOrientation position = new HeadOrientation(xOutput, yOutput);
        setHeadPosition(position);
    }
}
