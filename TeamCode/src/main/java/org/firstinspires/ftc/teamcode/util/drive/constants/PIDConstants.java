package org.firstinspires.ftc.teamcode.util.drive.constants;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.lib.PIDParams;

@Config
public class PIDConstants {
    private static final double WHEEL_DIAMETER = 3.44488188976378;
    private static final double TICKS_PER_REVOLUTE = 537.7 * 26 / 20;
    public static final double TICKS_PER_INCH = TICKS_PER_REVOLUTE / (WHEEL_DIAMETER * Math.PI);


    public static double TargetAngle = -2;
    public static double MaxTargetAngle = 6;
    public static double MaxAngle = 60;

    public static boolean manualDrive = false;

    public static boolean MotorPIDEnabled = false;

    public static double DriveVelMin = 0.15;

    public static double TargetAngleMargin = 4;

    public static double HeadAngleConversion = 1;

    public static double LoopSpeedRatio = 4;

    public static double UprightPowerMargin = 50;

    public static double LargeAnglePIDMargin = 20;

    public static PIDParams SmallAnglePID = new PIDParams(0.08, 0.00005, 1, 0.1); // 0.06, 0.0002, 1
    public static PIDParams LargeAnglePID = new PIDParams(0.1, 0.0001, 1, 0.2);
    public static PIDParams VelPID = new PIDParams(-0.03, 0.000, 0.2, 0.3); //-0.08, 0, 0

    public static PIDParams MotorPID = new PIDParams(0.0012, 0.000, 0, 0.3);
}
