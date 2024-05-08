package org.firstinspires.ftc.teamcode.util.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SpeedConstants {
    public static double ManualDrive = 0.5;
    public static double ManualAccel = 0.1;
    public static double ManualAccelDown = 0.1;
    public static double ManualTurn = 0.5;

    public static double ManualDriveMargin = 0.1;
    public static double DriveAngle = 6;

    public static double BrakeP = -0.0005;
    public static double BrakeError = 2;

    public static double HoldPower = 0.001;
}
