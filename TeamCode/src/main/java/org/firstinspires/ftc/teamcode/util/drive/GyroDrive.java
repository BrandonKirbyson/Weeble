package org.firstinspires.ftc.teamcode.util.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.opmodes.drive.Drive;
import org.firstinspires.ftc.teamcode.util.lib.PIDConstants;

public class GyroDrive {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private DriveState state = DriveState.STOPPED;

    private final PIDController anglePID = new PIDController(BalanceConstants.AnglePID);
    private final PIDController velPID = new PIDController(BalanceConstants.IdleVelPID);

    private YawPitchRollAngles angles;

    private double targetVel = 0;
    private double targetAngle = BalanceConstants.TargetAngle;
    private double rotSpeed = 0;

    public GyroDrive(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initIMU(IMU imu) {
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );
    }

    public void drive(double drivePower, double turnPower) {
        targetVel = drivePower * SpeedConstants.ManualDrive;
        rotSpeed = turnPower * SpeedConstants.ManualTurn;
    }

    public void update(YawPitchRollAngles angles) {
        this.angles = angles;

        if (!isBalanced()) {
            stopMotors();
            return;
        }

        double currentVel = getVel();
        double currentAngle = angles.getPitch(AngleUnit.DEGREES);

        double velError = targetVel - currentVel;
        PIDConstants oldConstants = velPID.getConstants();
        if (Math.abs(velError) > BalanceConstants.DriveVelMin) {
            velPID.setConstants(BalanceConstants.DriveVelPID);
        } else {
            velPID.setConstants(BalanceConstants.IdleVelPID);
        }
        if (!oldConstants.equals(velPID.getConstants())) {
            velPID.reset();
        }
        if (Math.abs(targetAngle - currentAngle) < BalanceConstants.AngleMargin)
            targetAngle -= velPID.update(velError);

        double angleError = targetAngle - currentAngle;
        double outputPower = anglePID.update(angleError);

        setPower(outputPower + rotSpeed, outputPower - rotSpeed);

        if (Drive.DEBUG) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Angle", currentAngle);
            packet.put("AngleError", currentAngle);
            packet.put("AngleTarget", targetAngle);
            packet.put("Velocity", currentVel);
            packet.put("VelocityError", velError);
            packet.put("VelocityTarget", targetVel);
            packet.put("Turn", rotSpeed);
            packet.put("Output", outputPower);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    private double getVel() {
        return (leftMotor.getPower() + rightMotor.getPower()) / 2;
    }

    private void setPower(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public boolean isBalanced() {
        return Math.abs(angles.getPitch(AngleUnit.DEGREES) - BalanceConstants.TargetAngle) < BalanceConstants.MaxAngle;
    }

    public void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public DriveState getState() {
        return state;
    }
}
