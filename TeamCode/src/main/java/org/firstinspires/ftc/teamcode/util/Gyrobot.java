package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.constants.BalanceConstants;
import org.firstinspires.ftc.teamcode.util.constants.SpeedConstants;

public class Gyrobot {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private final IMU imu;

    private double previousError = 0;
    private double previousTime = 0;
    private double i = 0;

    private double drivePower = 0;
    private double turnPower = 0;

    public Gyrobot(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );
    }

    public void drive(double drivePower, double turnPower) {
        this.drivePower = drivePower * SpeedConstants.ManualDrive;
        this.turnPower = turnPower * SpeedConstants.ManualTurn;
    }

    public void update(Telemetry telemetry) {
        double angle = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);

        double currentTime = System.currentTimeMillis();

        double currentError = angle - BalanceConstants.TargetAngle;

        double p = BalanceConstants.Kp * currentError;

        i += BalanceConstants.Ki * (currentError * (currentTime - previousTime));
        i = Math.max(Math.min(i, BalanceConstants.MaxI), -BalanceConstants.MaxI);

        double d = BalanceConstants.Kd * (currentError - previousError) / (currentTime - previousTime);
        double output = p + i + d;

        previousError = currentError;
        previousTime = currentTime;

        if (Math.abs(angle) < BalanceConstants.MaxAngle) {
            leftMotor.setPower(output + drivePower - turnPower);
            rightMotor.setPower(output + drivePower + turnPower);
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        telemetry.addData("Angle", angle);
        telemetry.addData("Correction Speed", output);
    }
}
