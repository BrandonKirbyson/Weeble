package org.firstinspires.ftc.teamcode.util.constants;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.lib.PIDConstants;

@Config
public class BalanceConstants {
    public static double TICKS_PER_DEGREE = 537.7 / 360;

    public static double TargetAngle = 0;
    public static double MaxAngle = 30;
    public static double BalancedMargin = 2;

    public static double SmallPIDMargin = 30;

    public static double AngularP = 0.005;
    public static double MinAngular = 1;
    public static double MaxAngular = 1.5;

    public static PIDConstants SmallPID = new PIDConstants(0.04, 0.0002, 0.8);
    public static PIDConstants LargePID = new PIDConstants(0.03, 0.001, 1);

    public static double MaxI = 0.3;
}
