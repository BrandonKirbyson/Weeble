package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Weeble;
import org.firstinspires.ftc.teamcode.util.drive.DriveType;
import org.firstinspires.ftc.teamcode.util.head.HeadConstants;
import org.firstinspires.ftc.teamcode.util.lib.FtcDashboardManager;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;

@Config
@TeleOp(name = "EasyDrive")
public class EasyDrive extends LinearOpMode {
    protected DriveType getDriveType() {
        return DriveType.SMOOTH;
    }

    @Override
    public void runOpMode() {
        Weeble robot = new Weeble(hardwareMap, getDriveType());

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);
        StatefulGamepad gamepad2Buttons = new StatefulGamepad(gamepad2);

        waitForStart();

        robot.head.reset();

        double lastTime = System.currentTimeMillis();

        while (opModeIsActive() && !isStopRequested()) {
            double currentTime = System.currentTimeMillis();
            FtcDashboardManager.addData("Loop Time", currentTime - lastTime);
            lastTime = currentTime;

            gamepad1Buttons.update();
            gamepad2Buttons.update();

            if (!gamepad1Buttons.getButton(GamepadButton.RIGHT_BUMPER)) {
                robot.drive.drive(Math.max(Math.min(-gamepad1.left_stick_y + -gamepad2.left_stick_y, 1), -1), Math.max(Math.min(gamepad1.left_stick_x + gamepad2.left_stick_x, 1), -1), gamepad2Buttons.getButton(GamepadButton.LEFT_BUMPER));
            } else {
                robot.drive.drive(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad1Buttons.getButton(GamepadButton.LEFT_BUMPER));
            }

            if (gamepad2Buttons.getButton(GamepadButton.RIGHT_BUMPER)) {
                robot.drive.brake();
            }

            FtcDashboardManager.addData("Drive Type", robot.drive.getDriveType());

            if (!gamepad1Buttons.getButton(GamepadButton.RIGHT_BUMPER)) {
                //noinspection SuspiciousNameCombination
                robot.head.manualControl(gamepad1.right_stick_y, gamepad1.right_stick_x);
            }

            if (robot.drive.getTurnVelocity() == 0) {
                robot.head.setEyes(0.5 + (-gamepad1.left_trigger + gamepad1.right_trigger) / 2);
            }

            if (gamepad1Buttons.wasJustPressed(GamepadButton.DPAD_UP)) {
                robot.head.setEyebrows(HeadConstants.eyebrowsAngry);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.DPAD_DOWN)) {
                robot.head.setEyebrows(HeadConstants.eyebrowsSad);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.DPAD_LEFT)) {
                robot.head.setEyebrows(HeadConstants.eyebrowsNeutral);
            }

            robot.update();

            if (gamepad1Buttons.getButton(GamepadButton.RIGHT_BUMPER) && (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0)) {
                robot.arms.setLeftArmPosition(gamepad1.left_stick_y);
                robot.arms.setRightArmPosition(gamepad1.right_stick_y);
            }

            FtcDashboardManager.update();
        }
    }
}