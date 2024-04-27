package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.util.constants.BalanceConstants;
import org.firstinspires.ftc.teamcode.util.constants.SpeedConstants;
import org.firstinspires.ftc.teamcode.util.lib.PIDConstants;

public class Gyrobot {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    public final Head head;

    private final IMU imu;

    private double previousError = 0;
    private double previousTime = 0;
    private double i = 0;

    private boolean brake = false;

    private PIDConstants lastPID;

    private boolean idle;

    private YawPitchRollAngles angles;

    public Gyrobot(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );

        head = new Head(hardwareMap);
    }

    public void update(Telemetry telemetry) {
        angles = imu.getRobotYawPitchRollAngles();
        head.updateAngles(angles);

        if (Drive.DEBUG) {
//            telemetry.addData("Angle", angles.getPitch(AngleUnit.DEGREES));
//            telemetry.addData("Angular Velocity", imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Angle", angles.getPitch(AngleUnit.DEGREES));
            packet.put("Angular Velocity", imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    public boolean isBalanced() {
        return Math.abs(angles.getPitch(AngleUnit.DEGREES) - BalanceConstants.TargetAngle) < BalanceConstants.MaxAngle;
    }

    public void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void drive(double drivePower) {
        brake = true;

        double currentTime = System.currentTimeMillis();
        double driveAngle = drivePower * SpeedConstants.DriveAngle;
        double currentError = angles.getPitch(AngleUnit.DEGREES) - BalanceConstants.TargetAngle + driveAngle;

        PIDConstants pid = BalanceConstants.DrivePID;
        if ((currentError < 0 && drivePower > 0) || (currentError > 0 && drivePower < 0)) {
            pid = BalanceConstants.AcceleratePID;
        }
        double output = getPIDOutput(pid, currentError, previousError, currentTime - previousTime);

        leftMotor.setPower(output);
        rightMotor.setPower(output);

        previousError = currentError;
        previousTime = currentTime;
    }

    public void turn(double turnPower) {
        leftMotor.setPower(leftMotor.getPower() - turnPower * SpeedConstants.ManualTurn);
        rightMotor.setPower(rightMotor.getPower() + turnPower * SpeedConstants.ManualTurn);
    }


    public void idle() {
        double currentTime = System.currentTimeMillis();
        double currentError = angles.getPitch(AngleUnit.DEGREES) - BalanceConstants.TargetAngle;

        PIDConstants pid = Drive.AnyTerrain ? BalanceConstants.IdleAnyTerrainPID : BalanceConstants.IdlePID;
        if (Math.abs(currentError) > BalanceConstants.MaxAngle) {
            pid = BalanceConstants.UprightPID;
        }
        double output = getPIDOutput(pid, currentError, previousError, currentTime - previousTime);
        if (brake) {
            output += currentError * SpeedConstants.BrakeP;
            if (currentError < SpeedConstants.BrakeError) {
                brake = false;
            }
        }

        leftMotor.setPower(output);
        rightMotor.setPower(output);

        previousError = currentError;
        previousTime = currentTime;
    }

    private double getPIDOutput(PIDConstants pid, double error, double previousError, double deltaTime) {
        double p = pid.Kp * error;
        if (!pid.equals(lastPID)) {
            i = 0;
        }
        i += pid.Ki * error * deltaTime;
        i = Math.max(Math.min(i, pid.MaxI), -pid.MaxI);
        double d = pid.Kd * (error - previousError) / deltaTime;
        lastPID = pid;
        return p + i + d;
    }
}
