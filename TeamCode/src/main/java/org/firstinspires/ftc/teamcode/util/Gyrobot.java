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
import org.firstinspires.ftc.teamcode.util.constants.BalanceConstants;
import org.firstinspires.ftc.teamcode.util.constants.SpeedConstants;
import org.firstinspires.ftc.teamcode.util.lib.PIDConstants;

public class Gyrobot {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    public final Eyes eyes;

    private final IMU imu;

    private double previousError = 0;
    private double previousTime = 0;
    private double i = 0;

    private double drivePower = 0;
    private double turnPower = 0;

    private boolean stopped = false;

    private double leftTargetPos = 0;
    private double rightTargetPos = 0;

    private double lastLeftPos = 0;
    private double lastRightPos = 0;

    public Gyrobot(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        eyes = new Eyes(hardwareMap, imu);
    }

    public void drive(double drivePower, double turnPower) {
        if ((this.drivePower != 0 && drivePower == 0) || (this.turnPower != 0 && turnPower == 0)) {
            leftTargetPos = leftMotor.getCurrentPosition();
            rightTargetPos = rightMotor.getCurrentPosition();
        }
        this.drivePower = drivePower * SpeedConstants.ManualDrive;
//        drivePower = drivePower > SpeedConstants.ManualDriveMargin ? SpeedConstants.ManualDrive : drivePower < -SpeedConstants.ManualDriveMargin ? -SpeedConstants.ManualDrive : 0;
        if (drivePower != 0) {
            if (this.drivePower > drivePower) {
                this.drivePower = Math.max(this.drivePower - SpeedConstants.ManualAccel, drivePower);
            } else if (this.drivePower < drivePower) {
                this.drivePower = Math.min(this.drivePower + SpeedConstants.ManualAccel, drivePower);
            }
//            this.drivePower = drivePower;
        } else {
//            if (this.drivePower > 0) {
//                this.drivePower = Math.max(this.drivePower - SpeedConstants.ManualAccelDown, 0);
//            } else if (this.drivePower < 0) {
//                this.drivePower = Math.min(this.drivePower + SpeedConstants.ManualAccelDown, 0);
//            }
            if (stopped) {
                this.drivePower = 0;
            }
            stopped = true;
            this.drivePower = -this.drivePower;
        }
        this.turnPower = turnPower * SpeedConstants.ManualTurn;
    }

    public void update(Telemetry telemetry) {
        double currentTime = System.currentTimeMillis();

        double angle = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        double angleVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate;

        boolean smallPID = drivePower == 0;

//        double targetAngle = BalanceConstants.TargetAngle + (!smallPID ? (drivePower > 0 ? SpeedConstants.DriveAngle : -SpeedConstants.DriveAngle) : 0);
        double targetAngle = BalanceConstants.TargetAngle;
        double currentError = angle - targetAngle;

        boolean balanced = Math.abs(currentError) < BalanceConstants.BalancedMargin;

//        boolean smallPID = Math.abs(currentError) < BalanceConstants.SmallPIDMargin;

        PIDConstants pid = smallPID ? BalanceConstants.SmallPID : BalanceConstants.LargePID;

        double p = pid.Kp * currentError;

//        p *= Math.min(Math.max(BalanceConstants.AngularP * Math.abs(angleVelocity), BalanceConstants.MinAngular), BalanceConstants.MaxAngular);

        i += pid.Ki * (currentError * (currentTime - previousTime));
        i = Math.max(Math.min(i, BalanceConstants.MaxI), -BalanceConstants.MaxI);

        double d = pid.Kd * (currentError - previousError) / (currentTime - previousTime);
        double output = p + i + d;

        double balanceTicks = (previousError - currentError) * BalanceConstants.TICKS_PER_DEGREE;

        double leftPos = leftMotor.getCurrentPosition();
        double rightPos = rightMotor.getCurrentPosition();

        leftTargetPos += balanceTicks;
        rightTargetPos += balanceTicks;

        double leftError = leftTargetPos - leftPos;
        double rightError = rightTargetPos - rightPos;

//        double holdLeftPower = leftError * SpeedConstants.HoldPower;
//        double holdRightPower = rightError * SpeedConstants.HoldPower;
        double holdLeftPower = 0;
        double holdRightPower = 0;

        previousError = currentError;
        previousTime = currentTime;
        lastLeftPos = leftPos;
        lastRightPos = rightPos;

        if (Math.abs(angle) < BalanceConstants.MaxAngle) {
            if (drivePower != 0) {
                leftMotor.setPower(output + drivePower - turnPower);
                rightMotor.setPower(output + drivePower + turnPower);
            } else {
                leftMotor.setPower(output + holdLeftPower - turnPower);
                rightMotor.setPower(output + holdRightPower + turnPower);
            }
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        telemetry.addData("Angle", angle);
        telemetry.addData("Angular Velocity", angleVelocity);
        telemetry.addData("Correction Speed", output);
        telemetry.addData("Drive Power", drivePower);
        telemetry.addData("Left Pos", leftPos);
        telemetry.addData("Right Pos", rightPos);
        telemetry.addData("Left Target Pos", leftTargetPos);
        telemetry.addData("Right Target Pos", rightTargetPos);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Angle", angle);
        packet.put("Angular Velocity", angleVelocity);
        packet.put("Correction Speed", output);
        packet.put("Left Pos", leftPos);
        packet.put("Right Pos", rightPos);
        packet.put("Left Target Pos", leftTargetPos);
        packet.put("Right Target Pos", rightTargetPos);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
