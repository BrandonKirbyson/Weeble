package org.firstinspires.ftc.teamcode.util.drive.constants;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.lib.PIDParams;

@Config
public class BalanceConstants {
    public static double TargetAngle = -3;
    public static double HeadAngleModifier = -15;

    public static double MaxAngle = 60;

    public static double MaxPlaceAngle = 20;
    public static double PlaceDelay = 0.5;

    public static boolean UpdateAngle = true;
    public static boolean AngleAssistedDriving = true;
    public static PIDParams AnglePID = new PIDParams(0.002, 0.0, 0.0, 0.3);
    public static double MaxTargetAngle = 10;
}
