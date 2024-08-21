package org.firstinspires.ftc.teamcode.util.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LQRConstants {
    private final static double GRAVITY = 9.81; // m/s^2
    public static double LENGTH = 0.04708; // length to center of mass (m)
    public static double MASS = 3.401942775; // kg (7.5 lbs)
    public static double WHEEL_RADIUS = 0.05; // m
    public static double TICKS_PER_REVOLUTE = 537.7 * 26 / 20;

    public static final double TICKS_PER_M = TICKS_PER_REVOLUTE / (WHEEL_RADIUS * Math.PI);

    public static double[][] getA() {
        return new double[][]{
                {0, 1, 0, 0},
                {GRAVITY / LENGTH, 0, 0, 0},
                {0, 0, 0, 1},
                {-GRAVITY / MASS, 0, 0, 0}
        };
    }

    public static double[][] getB() {
        return new double[][]{
                {0},
                {-1 / (MASS * (LENGTH * LENGTH))},
                {0},
                {1 / MASS}
        };
    }

    public static double AnglePenalty = 1;
    public static double AngularVelocityPenalty = 0.0;
    public static double PositionPenalty = 0.02;
    public static double VelocityPenalty = 1;

    //    public static double[] Q = {AnglePenalty, AngularVelocityPenalty, PositionPenalty, VelocityPenalty}; // state penalty, x, x dot, theta, theta dot
    public static double[] getQ() {
        return new double[]{AnglePenalty, AngularVelocityPenalty, PositionPenalty, VelocityPenalty}; // state penalty, x, x dot, theta, theta dot
    }

    public static double R = 0.1;

    public static double TargetAngle = -3.5;

    public static boolean UpdateLQRGains = false;
}
