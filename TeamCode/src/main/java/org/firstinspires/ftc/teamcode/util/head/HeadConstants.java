package org.firstinspires.ftc.teamcode.util.head;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.lib.PIDParams;

@Config
public class HeadConstants {
    public static double xCenter = 0.55;
    public static double xMin = -15;
    public static double xMax = 20;
    public static double xConversion = 180;

    public static double yCenter = 0.42;
    public static double yMin = -70;
    public static double yMax = 80;
    public static double yConversion = -180;

    public static double eyesCenter = 0.5;
    public static double eyesMin = -35;
    public static double eyesMax = 45;
    public static double eyesConversion = 180;

    public static double eyebrowsAngry = 0.75;
    public static double eyebrowsSad = 0.4;
    public static double eyebrowsNeutral = 0.6;

    public static double trackingTimeout = 1;

    public static boolean eyesTracking = true;

    public static PIDParams xTrackingPID = new PIDParams(-0.002, 0.0, 0.0, 0.3);
    public static PIDParams yTrackingPID = new PIDParams(0.008, 0.0, 0.0, 0.3);
    public static PIDParams eyesTrackingPID = new PIDParams(0.008, 0.0, 0.0, 0.3);

    public static double HeadAutoTurnAmount = 0;
    public static double EyesAutoTurnAmount = 120;

    public static double xSpeed = 1;
    public static double ySpeed = 1;

    public static boolean HoldX = true;
}