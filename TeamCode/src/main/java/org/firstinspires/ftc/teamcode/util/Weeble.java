package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.arms.ArmPosition;
import org.firstinspires.ftc.teamcode.util.arms.Arms;
import org.firstinspires.ftc.teamcode.util.drive.DriveState;
import org.firstinspires.ftc.teamcode.util.drive.GyroDrive;
import org.firstinspires.ftc.teamcode.util.head.Head;
import org.firstinspires.ftc.teamcode.util.overlay.OverlayManager;
import org.firstinspires.ftc.teamcode.util.vision.Vision;

public class Weeble {
    public final Vision vision;
    public final GyroDrive drive;
    public final Arms arms;
    public final Head head;

    private final IMU imu;

    private final OverlayManager overlay = new OverlayManager();

    public Weeble(HardwareMap hardwareMap) {
        vision = new Vision(hardwareMap);
        drive = new GyroDrive(hardwareMap);
        arms = new Arms(hardwareMap);
        head = new Head(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        drive.initIMU(imu);
    }

    public void update() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        drive.update(angles);
        head.updateAngles(angles);

        vision.update();

        head.update(vision.getTrackingCenter());

        if (drive.getLastState() != DriveState.STOPPED && drive.getState() == DriveState.STOPPED) {
            arms.setArmPosition(ArmPosition.Forward);
            head.protectionMode();
        } else if (drive.getLastState() == DriveState.STOPPED && drive.getState() != DriveState.STOPPED) {
            arms.setArmPosition(ArmPosition.Down);
            head.reset();
        }

        overlay.updatePose(drive.getPose());
        overlay.updatePoints(vision.sensorMapping.getPoints());

        overlay.update();
    }
}
