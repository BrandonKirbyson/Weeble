package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.util.head.Head;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;

@TeleOp(name = "EyesTest", group = "tests")
public class EyesTest extends LinearOpMode {
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

            if (!gamepad1Buttons.getButton(GamepadButton.Y)) {
                head.setManualPosition(gamepad1.right_stick_y, gamepad1.right_stick_x);
            } else {
                head.holdPosition();
            }
        }
    }
}