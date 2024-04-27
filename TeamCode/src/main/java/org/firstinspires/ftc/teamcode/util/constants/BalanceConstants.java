package org.firstinspires.ftc.teamcode.util.constants;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.lib.PIDConstants;

@Config
public class BalanceConstants {
    public static double TICKS_PER_DEGREE = 537.7 / 360;

    public static double TargetAngle = 0;
    public static double MaxAngle = 30;

    public static PIDConstants IdlePID = new PIDConstants(0.06, 0.0004, 0.8, 0.1);
    public static PIDConstants UprightPID = new PIDConstants(0.1, 0.0, 0.8, 0.1);
    public static PIDConstants DrivePID = new PIDConstants(0.03, 0.001, 1, 0.3);
    public static PIDConstants AcceleratePID = new PIDConstants(0.03, 0.001, 1, 0.3);
}
