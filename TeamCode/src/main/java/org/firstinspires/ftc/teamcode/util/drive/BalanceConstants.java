package org.firstinspires.ftc.teamcode.util.drive;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.lib.PIDConstants;

@Config
public class BalanceConstants {
    private static final double WHEEL_DIAMETER = 3.44488188976378;
    private static final double TICKS_PER_REVOLUTE = 537.7 * 26 / 20;
    public static final double TICKS_PER_INCH = TICKS_PER_REVOLUTE / (WHEEL_DIAMETER * Math.PI);

    public static double TargetAngle = 0;
    public static double MaxAngle = 60;

    public static boolean manualDrive = false;

    public static boolean MotorPIDEnabled = false;

    public static double DriveVelMin = 0.15;

    public static double LoopSpeedRatio = 5;

    public static double UprightPowerMargin = 50;

    public static double LargeAnglePIDMargin = 4;

    public static PIDConstants SmallAnglePID = new PIDConstants(0.08, 0.0000, 1, 0.3); // 0.06, 0.0002, 1
    public static PIDConstants LargeAnglePID = new PIDConstants(0.08, 0.0000, 0.5, 0.5);
    public static PIDConstants VelPID = new PIDConstants(-0.05, 0.000, 0.15, 0.3); //-0.08, 0, 0

    public static PIDConstants MotorPID = new PIDConstants(0.0012, 0.000, 0, 0.3);
}
