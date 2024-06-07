package org.firstinspires.ftc.teamcode.util.head;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.lib.PIDConstants;

@Config
public class HeadConstants {
    public static double xCenter = 0.5;
    public static double xMin = -45;
    public static double xMax = 45;
    public static double xConversion = 180;

    public static double yCenter = 0.5;
    public static double yMin = -90;
    public static double yMax = 90;
    public static double yConversion = 180;

    public static double eyesCenter = 0.5;
    public static double eyesMin = -45;
    public static double eyesMax = 45;
    public static double eyesConversion = 180;

    public static PIDConstants xTrackingPID = new PIDConstants(0.02, 0.0, 0.0, 0.3);
    public static PIDConstants yTrackingPID = new PIDConstants(0.02, 0.0, 0.0, 0.3);
}