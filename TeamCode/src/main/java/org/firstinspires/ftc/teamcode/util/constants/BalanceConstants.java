package org.firstinspires.ftc.teamcode.util.constants;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.lib.PIDConstants;

@Config
public class BalanceConstants {
    public static double TICKS_PER_DEGREE = 537.7 / 360;

    public static double TargetAngle = 2;
    public static double MaxAngle = 30;
    public static double BalancedMargin = 2;

    public static double SmallPIDMargin = 30;

    public static PIDConstants SmallPID = new PIDConstants(0.04, 0.0006, 1.2);
    public static PIDConstants LargePID = new PIDConstants(0.06, 0.001, 1);

    public static double MaxI = 0.3;
}
