package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.util.head.Head;
import org.firstinspires.ftc.teamcode.util.head.HeadPresets;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;

@TeleOp(name = "HeadTest", group = "tests")
public class HeadTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );

        Head head = new Head(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();

            head.updateAngles(imu.getRobotYawPitchRollAngles());

            if (gamepad1.left_stick_y != 0 && gamepad2.left_stick_x != 0) {
                head.manualControl(gamepad1.left_stick_y, gamepad1.left_stick_x);
            }

            if (gamepad1Buttons.wasJustPressed(GamepadButton.Y)) {
                head.runAnimation(HeadPresets.NodYes);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.X)) {
                head.runAnimation(HeadPresets.ShakeNo);
            }
        }
    }
}