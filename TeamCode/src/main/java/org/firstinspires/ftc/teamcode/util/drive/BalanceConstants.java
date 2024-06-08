package org.firstinspires.ftc.teamcode.util.drive;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.lib.PIDConstants;

@Config
public class BalanceConstants {
    public static double TICKS_PER_REVOLUTE = 537.7 * 26 / 20;

    public static double TargetAngle = 0;
    public static double MaxAngle = 30;

    public static boolean MotorPIDEnabled = false;

    public static double AngleMargin = 2;

    public static double DriveVelMin = 0.15;

    public static double VelErrorMargin = 0.1;

    public static double LoopSpeedRatio = 10;

    public static PIDConstants AnglePID = new PIDConstants(0.06, 0.0002, 1, 0.3);
    public static PIDConstants DriveVelPID = new PIDConstants(0.3, 0, 0.8, 0.3);
    public static PIDConstants IdleVelPID = new PIDConstants(-0.2, 0.000, 0, 0.3);

    public static PIDConstants MotorPID = new PIDConstants(0.0012, 0.000, 0, 0.3);
}
