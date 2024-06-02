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

public class GyroDrive {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private DriveState lastState = DriveState.STOPPED;
    private DriveState state = DriveState.STOPPED;

    private final PIDController anglePID = new PIDController(BalanceConstants.AnglePID);
    private final PIDController velPID = new PIDController(BalanceConstants.IdleVelPID);

    private final PIDController leftMotorPID = new PIDController(BalanceConstants.MotorPID);
    private final PIDController rightMotorPID = new PIDController(BalanceConstants.MotorPID);

    private YawPitchRollAngles angles;

    private double lastLeftTicks = 0;
    private double lastRightTicks = 0;

    private int loopCounter = 0;

    private double leftSpeed = 0;
    private double rightSpeed = 0;

    private boolean lastBalanced = false;
    private double lastDrivePower;

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

    private void updateAngle(TelemetryPacket packet) {
        double leftTicks = leftMotor.getCurrentPosition();
        double rightTicks = rightMotor.getCurrentPosition();
        leftSpeed = leftTicks - lastLeftTicks;
        rightSpeed = rightTicks - lastRightTicks;

        double currentVel = getVel();
        double velError = targetVel - currentVel;

        targetAngle += velPID.update(velError);

        packet.put("Velocity", currentVel);
        packet.put("VelocityError", velError);
        packet.put("VelocityTarget", targetVel);

        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
    }

    public void update(YawPitchRollAngles angles) {
        TelemetryPacket packet = new TelemetryPacket();
        this.angles = angles;

        if (!isBalanced()) {
            stopMotors();
            state = DriveState.STOPPED;
            lastBalanced = false;
            return;
        } else if (!lastBalanced) {
            targetAngle = BalanceConstants.TargetAngle;
            lastBalanced = true;
        }

        if (loopCounter % BalanceConstants.LoopSpeedRatio == 0) {
            updateAngle(packet);
        }

        loopCounter++;

        double currentAngle = angles.getPitch(AngleUnit.DEGREES);

        double angleError = targetAngle - currentAngle;
        double outputPower = anglePID.update(angleError);

        setPower(outputPower + rotSpeed, outputPower - rotSpeed);

        updateState();

        if (Drive.DEBUG) {
            packet.put("Angle", currentAngle);
            packet.put("AngleError", currentAngle);
            packet.put("AngleTarget", targetAngle);
            packet.put("Turn", rotSpeed);
            packet.put("Output", outputPower);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    private void updateState() {
        lastState = state;
        if (isBalanced()) {
            if (targetVel == 0) {
                if (getVel() < BalanceConstants.DriveVelMin) {
                    state = DriveState.IDLE;
                } else {
                    state = DriveState.DRIVING;
                }
            } else {
                state = DriveState.DRIVING;
            }
        } else {
            state = DriveState.STOPPED;
        }
    }

    private double getVel() {
        return (leftSpeed + rightSpeed) / 2;
    }

    private void setPower(double leftPower, double rightPower) {
        double leftError = leftPower * BalanceConstants.TICKS_PER_REVOLUTE - leftSpeed;
        double rightError = rightPower * BalanceConstants.TICKS_PER_REVOLUTE - rightSpeed;
        double leftOutput = leftMotorPID.update(leftError);
        double rightOutput = rightMotorPID.update(rightError);

        leftMotor.setPower(leftOutput);
        rightMotor.setPower(rightOutput);
    }

    public boolean isBalanced() {
        return Math.abs(angles.getPitch(AngleUnit.DEGREES) - BalanceConstants.TargetAngle) < BalanceConstants.MaxAngle;
    }

    public void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public DriveState getLastState() {
        return lastState;
    }

    public DriveState getState() {
        return state;
    }
}
