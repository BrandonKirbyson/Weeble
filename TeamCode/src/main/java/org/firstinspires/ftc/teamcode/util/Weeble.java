package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.drive.GyroDrive;

public class Weeble {
    //    public final Vision vision;
    public final GyroDrive drive;
//    public final Arms arms;
//    public final Head head;

    private final IMU imu;

    public Weeble(HardwareMap hardwareMap) {
//        vision = new Vision(hardwareMap);
        drive = new GyroDrive(hardwareMap);
//        arms = new Arms(hardwareMap);
//        head = new Head(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        drive.initIMU(imu);
    }

    public void update() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        drive.update(angles);
//        vision.update();


//        head.update();

//        if (drive.getLastState() == DriveState.DRIVING && drive.getState() == DriveState.STOPPED) {
//            arms.setArmPosition(ArmPosition.Forward);
//        } else if (drive.getLastState() == DriveState.STOPPED && drive.getState() == DriveState.DRIVING) {
//            arms.setArmPosition(ArmPosition.Down);
//        }
    }
}
