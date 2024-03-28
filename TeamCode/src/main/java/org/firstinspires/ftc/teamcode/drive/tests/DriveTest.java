package org.firstinspires.ftc.teamcode.drive.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "DriveTest", group = "tests")
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "left");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "right");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
//            leftMotor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) * 0.25);
//            rightMotor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) * 0.25);
            double correctionSpeed = Math.max(Math.min((imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES) - 2) * 0.04, 0.3), -0.3);
            leftMotor.setPower(correctionSpeed);
            rightMotor.setPower(correctionSpeed);

            telemetry.addData("Angle", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            telemetry.addData("Correction Speed", correctionSpeed);
            telemetry.update();
        }
    }
}