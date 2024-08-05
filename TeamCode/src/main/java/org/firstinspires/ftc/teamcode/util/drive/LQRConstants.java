package org.firstinspires.ftc.teamcode.util.drive;

public class LQRConstants {
    private final static double GRAVITY = 9.81; // m/s^2
    private final static double LENGTH = 50; // length to center of mass (mm)
    private final static double MASS = 10; // pounds
    private final static double WHEEL_RADIUS = 100; // mm

    public static final double[][] A = {
            {0, 1, 0, 0},
            {GRAVITY / LENGTH, 0, 0, 0},
            {0, 0, 0, 1},
            {-GRAVITY / MASS, 0, 0, 0}
    }; // state matrix

    public static final double[][] B = {
            {1 / -(MASS * (LENGTH * LENGTH))},
            {0},
            {0},
            {1 / MASS}
    }; // control matrix

    public static double AnglePenalty = 1;
    public static double AngularVelocityPenalty = 0;
    public static double PositionPenalty = 0;
    public static double VelocityPenalty = 0;

    public static double[] Q = {AnglePenalty, AngularVelocityPenalty, PositionPenalty, VelocityPenalty}; // state penalty, x, x dot, theta, theta dot
    public static double R = 1;
}
