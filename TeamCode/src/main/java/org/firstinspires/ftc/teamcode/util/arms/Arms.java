package org.firstinspires.ftc.teamcode.util.arms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.drive.constants.PIDConstants;

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

    public void setLeftArmPosition(double position) {
        leftArm.setPosition(position);
    }

    public void setRightArmPosition(double position) {
        rightArm.setPosition(position);
    }

    public void update(double angle) {
        if (Math.abs(angle) > PIDConstants.MaxAngle) {
            if (angle < 0) {
                setArmPosition(ArmPosition.FullForward);
            } else {
                setArmPosition(ArmPosition.FullBack);
            }
        } else if (Math.abs(angle) > ArmPosition.MovingAngle) {
            if (angle < 0) {
                setArmPosition(ArmPosition.Forward);
            } else {
                setArmPosition(ArmPosition.Back);
            }
        } else {
            setArmPosition(ArmPosition.Down);
        }
    }
}
