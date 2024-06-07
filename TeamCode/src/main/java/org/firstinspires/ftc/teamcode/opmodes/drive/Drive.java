package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Weeble;
import org.firstinspires.ftc.teamcode.util.head.HeadPresets;
import org.firstinspires.ftc.teamcode.util.lib.FtcDashboardManager;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;
import org.firstinspires.ftc.teamcode.util.vision.VisionMode;

@Config
@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    public static boolean DEBUG = true;

    @Override
    public void runOpMode() {
        Weeble robot = new Weeble(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();

            robot.drive.drive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

            robot.update();

            if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0) {
                //noinspection SuspiciousNameCombination
                robot.head.manualControl(gamepad1.left_stick_y, gamepad1.left_stick_x);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.B)) {
                robot.vision.setMode(VisionMode.FACE_TRACKING);
                robot.head.setTracking(true);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.Y)) {
                robot.head.runAnimation(HeadPresets.NodYes);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.X)) {
                robot.head.runAnimation(HeadPresets.ShakeNo);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                robot.head.reset();
            }

//            if (gamepad1Buttons.getButton(GamepadButton.Y)) {
//                robot.head.holdPosition();
//            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
//                robot.head.reset();
//            } else if (gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0) {
//                headMoving = true;
//                robot.head.setManualPosition(gamepad1.right_stick_y, gamepad1.right_stick_x);
//            } else if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
//                robot.head.adjustSetPosition(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right);
//            } else if (headMoving) {
//                robot.head.reset();
//                headMoving = false;
//            }

//            robot.head.setEyes(gamepad1.right_trigger - gamepad1.left_trigger);
            FtcDashboardManager.update();
        }
    }
}