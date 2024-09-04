package org.firstinspires.ftc.teamcode.util.lib;

public class PIDParams {
    public double Kp;
    public double Ki;
    public double Kd;
    public double MaxI;

    public PIDParams(double p, double i, double d, double maxI) {
        Kp = p;
        Ki = i;
        Kd = d;
        MaxI = maxI;
    }

    public boolean equals(PIDParams pid) {
        if (pid == null) return false;
        return pid.Kp == Kp && pid.Ki == Ki && pid.Kd == Kd && pid.MaxI == MaxI;
    }
}
