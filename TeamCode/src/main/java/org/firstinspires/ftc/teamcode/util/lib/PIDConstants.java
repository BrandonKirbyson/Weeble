package org.firstinspires.ftc.teamcode.util.lib;

public class PIDConstants {
    public double Kp;
    public double Ki;
    public double Kd;

    public PIDConstants(double p, double i, double d) {
        Kp = p;
        Ki = i;
        Kd = d;
    }
}
