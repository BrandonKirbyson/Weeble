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
    private final Servo eyebrowsServo;

    private YawPitchRollAngles angles;

    private HeadOrientation currentPosition = new HeadOrientation(0, 0, 0);

    private double yPos = 0;

    private ArrayList<HeadAnimationKeyframe> currentAnimation = null;
    private final ElapsedTime animationTimer = new ElapsedTime();
    private int animationIndex = 0;

    private final PIDController xTrackingPID = new PIDController(HeadConstants.xTrackingPID);
    private final PIDController yTrackingPID = new PIDController(HeadConstants.yTrackingPID);
    private final PIDController eyesTrackingPID = new PIDController(HeadConstants.eyesTrackingPID);
    private boolean tracking = false;
    private final ElapsedTime trackingTimout = new ElapsedTime();

    private double lastPitch;

    private boolean manualControl = false;

    public Head(HardwareMap hardwareMap) {
        neckServo = hardwareMap.get(Servo.class, "neck");
        headServo = hardwareMap.get(Servo.class, "head");
        eyesServo = hardwareMap.get(Servo.class, "eyes");
        eyebrowsServo = hardwareMap.get(Servo.class, "eyebrows");

        lastPitch = neckServo.getPosition();

        setEyebrows(HeadConstants.eyebrowsNeutral);
    }

    public void setEyebrows(double position) {
        eyebrowsServo.setPosition(position);
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

    public double getDeltaPitch() {
        double deltaPitch = neckServo.getPosition() - lastPitch;
        lastPitch = neckServo.getPosition();
        return deltaPitch;
    }

    public double getPitch() {
        return neckServo.getPosition();
    }

    public void update(Point trackingTarget) {
        updateCurrentHeadOrientation();

        if (currentAnimation != null) {
            updateAnimation();
        } else if (tracking) {
            trackObject(trackingTarget);
        }
    }

    public void initHoldHead() {
        yPos = angles.getYaw(AngleUnit.DEGREES);
    }

    public void holdHead() {
        HeadOrientation position = new HeadOrientation(0, 0, 0);
        position.y = -yPos;
        relativeToWorld(position);
        position.eyes = 0;
        if (HeadConstants.HoldX) position.x = 0;
        setHeadPosition(position);
    }

    public void autoTurn(double amount) {
        HeadOrientation position = new HeadOrientation(0, 0, 0);
        position.y = amount * HeadConstants.HeadAutoTurnAmount;
        position.eyes = amount * HeadConstants.EyesAutoTurnAmount;
        setEyes(position.eyes / HeadConstants.eyesConversion + HeadConstants.eyesCenter);
//        setHeadPosition(position);
    }

    public void reset() {
        HeadOrientation position = new HeadOrientation(0, 0, 0);
        setHeadPosition(position);
        tracking = false;
    }

    public void protectionMode() {
        HeadOrientation position = new HeadOrientation(0, 0, 0);
        if (angles.getPitch(AngleUnit.DEGREES) < 0) {
            position.x = 90;
        } else {
            position.x = -90;
        }
        setHeadPosition(position);
    }

    public void manualControl(double x, double y) {
        if (x != 0 || y != 0) {
            HeadOrientation position = new HeadOrientation(x * 90, y * 90);
            setHeadPosition(position);
            tracking = false;
            manualControl = true;
        } else if (manualControl) {
            reset();
            manualControl = false;
        }
    }

    public void manualControlAdjust(double x, double y) {
        if (x != 0 || y != 0) {
            HeadOrientation position = currentPosition;
            position.x += x * HeadConstants.xSpeed;
            position.y += y * HeadConstants.ySpeed;
            setHeadPosition(position);
            tracking = false;
        }
    }

    public void setEyes(double e) {
        eyesServo.setPosition(e);
    }

    public void setHeadPosition(HeadOrientation position) {
        neckServo.setPosition(position.x / HeadConstants.xConversion + HeadConstants.xCenter);
        headServo.setPosition(position.y / HeadConstants.yConversion + HeadConstants.yCenter);
        eyesServo.setPosition(position.eyes / HeadConstants.eyesConversion + HeadConstants.eyesCenter);
    }

    private void relativeToWorld(HeadOrientation position) {
        position.x = position.x - angles.getPitch(AngleUnit.DEGREES);
        position.y = position.y - angles.getYaw(AngleUnit.DEGREES);
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
        if (target == null) {
            if (trackingTimout.seconds() > HeadConstants.trackingTimeout) {
                HeadOrientation position = new HeadOrientation(0, 0);
                setHeadPosition(position);
            }
            return;
        }
        trackingTimout.reset();

        // Flipped x and y because it is 2d point to 3d rotational axis
        double xError = target.y - currentPosition.x;
        double yError = target.x - currentPosition.y;

        double xOutput = currentPosition.x + xTrackingPID.update(xError);
        double yOutput = currentPosition.y + yTrackingPID.update(yError);

        double eyesOutput = currentPosition.eyes;
        if (HeadConstants.eyesTracking && xOutput < HeadConstants.xMin && xOutput > HeadConstants.xMax) {
            double eyesError = target.x - currentPosition.eyes;
            eyesOutput = currentPosition.eyes + eyesTrackingPID.update(eyesError);
        }

        HeadOrientation position = new HeadOrientation(xOutput, yOutput, eyesOutput);
        setHeadPosition(position);
    }
}
