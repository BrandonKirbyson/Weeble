package org.firstinspires.ftc.teamcode.util.drive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.lib.FtcDashboardManager;

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

    private final Pose pose = new Pose();

    private double lastTime = 0;

    private double lastLeftTicks = 0;
    private double lastRightTicks = 0;

    private int loopCounter = 0;

    private double leftVel = 0;
    private double rightVel = 0;
    private double vel = 0;

    private boolean lastBalanced = false;

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

    public Pose getPose() {
        return pose;
    }

    public void drive(double drivePower, double turnPower) {
        targetVel = drivePower * SpeedConstants.ManualDrive;
        rotSpeed = turnPower * SpeedConstants.ManualTurn;
    }

    private void updateAngle() {
        double time = System.currentTimeMillis();
        double leftTicks = leftMotor.getCurrentPosition();
        double rightTicks = rightMotor.getCurrentPosition();

        double deltaTime = time - lastTime;

        double ticksToVel = (1000 / deltaTime) * (1 / BalanceConstants.TICKS_PER_REVOLUTE);

        leftVel = (leftTicks - lastLeftTicks) * ticksToVel;
        rightVel = (rightTicks - lastRightTicks) * ticksToVel;

        vel = (leftVel + rightVel) / 2;

        double velError = targetVel - vel;

        targetAngle += velPID.update(velError);

        FtcDashboardManager.addData("Velocity", vel);
        FtcDashboardManager.addData("VelocityError", velError);
        FtcDashboardManager.addData("VelocityTarget", targetVel);

        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;


        pose.x += Math.cos(angles.getYaw(AngleUnit.RADIANS)) * vel;
        pose.y += Math.sin(angles.getYaw(AngleUnit.RADIANS)) * vel;
        pose.heading = angles.getYaw(AngleUnit.DEGREES);

        lastTime = time;
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
            updateAngle();
        }

        loopCounter++;

        double currentAngle = angles.getPitch(AngleUnit.DEGREES);

        double angleError = targetAngle - currentAngle;
        double outputPower = anglePID.update(angleError);

        setPower(outputPower + rotSpeed, outputPower - rotSpeed);

        updateState();

        FtcDashboardManager.addData("Angle", currentAngle);
        FtcDashboardManager.addData("AngleError", currentAngle);
        FtcDashboardManager.addData("AngleTarget", targetAngle);
        FtcDashboardManager.addData("Turn", rotSpeed);
        FtcDashboardManager.addData("Output", outputPower);
        overlayRobot();
    }

    private void updateState() {
        lastState = state;
        if (isBalanced()) {
            if (targetVel == 0) {
                if (vel < BalanceConstants.DriveVelMin) {
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

    private void setPower(double leftPower, double rightPower) {
        if (BalanceConstants.MotorPIDEnabled) {
            double leftError = leftPower * BalanceConstants.TICKS_PER_REVOLUTE - leftVel;
            double rightError = rightPower * BalanceConstants.TICKS_PER_REVOLUTE - rightVel;
            double leftOutput = leftMotorPID.update(leftError);
            double rightOutput = rightMotorPID.update(rightError);
//
            leftMotor.setPower(leftOutput);
            rightMotor.setPower(rightOutput);
        } else {
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
        }
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

    private void overlayRobot() {
        FtcDashboardManager.getPacket().fieldOverlay()
                .setFill("white")
                .setStroke("green")
                .fillCircle(pose.x, pose.y, 5)
                .strokeLine(pose.x, pose.y, Math.cos(Math.toRadians(pose.heading)) * 4, Math.sin(Math.toRadians(pose.heading)) * 4);
    }
}
