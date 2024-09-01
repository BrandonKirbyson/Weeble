package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Weeble;
import org.firstinspires.ftc.teamcode.util.drive.BalanceConstants;
import org.firstinspires.ftc.teamcode.util.drive.DriveState;
import org.firstinspires.ftc.teamcode.util.head.HeadConstants;
import org.firstinspires.ftc.teamcode.util.lib.FtcDashboardManager;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;
import org.firstinspires.ftc.teamcode.util.vision.VisionMode;

@Config
@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() {
        Weeble robot = new Weeble(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);
        StatefulGamepad gamepad2Buttons = new StatefulGamepad(gamepad2);

        waitForStart();

        double lastTime = System.currentTimeMillis();

        while (opModeIsActive() && !isStopRequested()) {
            double currentTime = System.currentTimeMillis();
            FtcDashboardManager.addData("Loop Time", currentTime - lastTime);
            lastTime = currentTime;

            gamepad1Buttons.update();
            gamepad2Buttons.update();

            robot.drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1Buttons.getButton(GamepadButton.LEFT_BUMPER));

            if (gamepad1Buttons.getButton(GamepadButton.RIGHT_BUMPER)) {
//                robot.drive.resetTarget();
                robot.drive.setTargetPos();
            }

            if (gamepad1Buttons.getButton(GamepadButton.RIGHT_STICK_BUTTON)) {
                robot.drive.emergencyStop();
            }

            if (gamepad1Buttons.wasJustPressed(GamepadButton.LEFT_STICK_BUTTON) && robot.drive.getState() == DriveState.STOPPED) {
                robot.uprightWithArms();
            }

            if (gamepad1Buttons.wasJustPressed(GamepadButton.RIGHT_STICK_BUTTON)) {
                robot.drive.setTargetAngle(BalanceConstants.TargetAngle);
            }

//            if (gamepad2Buttons.wasJustPressed(GamepadButton.Y)) {
//                robot.vision.setRecording(true);
//            } else if (gamepad2Buttons.wasJustPressed(GamepadButton.X)) {
//                robot.vision.setRecording(false);
//            }
            FtcDashboardManager.addData("RECORDING", robot.vision.isRecording());

            //noinspection SuspiciousNameCombination
            robot.head.manualControl(gamepad1.right_stick_y, gamepad1.right_stick_x);

            robot.head.setEyes(0.5 + (-gamepad1.left_trigger + gamepad1.right_trigger) / 2);

            if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                robot.vision.setMode(VisionMode.DISABLED);
                robot.head.reset();
                robot.head.setEyebrows(HeadConstants.eyebrowsNeutral);
            }
            if (gamepad1Buttons.wasJustPressed(GamepadButton.B)) {
                robot.vision.setMode(VisionMode.BLOB_TRACKING);
                robot.head.setTracking(true);
            }
            if (gamepad1Buttons.wasJustPressed(GamepadButton.X)) {
                robot.vision.setMode(VisionMode.FACE_TRACKING);
                robot.head.setTracking(true);
            }

            if (gamepad1Buttons.wasJustPressed(GamepadButton.DPAD_UP)) {
                robot.head.setEyebrows(HeadConstants.eyebrowsAngry);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.DPAD_DOWN)) {
                robot.head.setEyebrows(HeadConstants.eyebrowsSad);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.DPAD_LEFT)) {
                robot.head.setEyebrows(HeadConstants.eyebrowsNeutral);
            }

//            else if (gamepad1Buttons.wasJustPressed(GamepadButton.Y)) {
//                robot.head.runAnimation(HeadPresets.NodYes);
//            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.X)) {
//                robot.head.runAnimation(HeadPresets.ShakeNo);
//            }

            robot.update();

            if (gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) {
                robot.arms.setLeftArmPosition(gamepad2.left_stick_y);
                robot.arms.setRightArmPosition(gamepad2.right_stick_y);
            }

            FtcDashboardManager.update();
        }
    }
}