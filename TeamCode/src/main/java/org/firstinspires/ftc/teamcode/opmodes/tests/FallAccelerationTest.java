package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.lib.FtcDashboardManager;

@TeleOp(name = "FallAccelerationTest", group = "tests")
public class FallAccelerationTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "left");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "right");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FtcDashboardManager.addData("Angle", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            FtcDashboardManager.addData("Acceleration", imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate);
            FtcDashboardManager.update();

            telemetry.addData("Angle", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}