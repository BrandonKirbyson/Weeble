package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Weeble;
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

        waitForStart();

        double lastTime = System.currentTimeMillis();

        while (opModeIsActive() && !isStopRequested()) {
            double currentTime = System.currentTimeMillis();
            FtcDashboardManager.addData("Loop Time", currentTime - lastTime);
            lastTime = currentTime;

            gamepad1Buttons.update();

            robot.drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x);

            //noinspection SuspiciousNameCombination
            robot.head.manualControl(gamepad1.right_stick_y, gamepad1.right_stick_x);

            robot.head.setEyes(0.5 + (gamepad1.left_trigger - gamepad1.right_trigger) / 2);

            if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                robot.vision.setMode(VisionMode.DISABLED);
                robot.head.reset();
            }
            if (gamepad1Buttons.wasJustPressed(GamepadButton.B)) {
                robot.vision.setMode(VisionMode.BLOB_TRACKING);
                robot.head.setTracking(true);
            }
            if (gamepad1Buttons.wasJustPressed(GamepadButton.LEFT_BUMPER)) {
                robot.vision.setMode(VisionMode.FACE_TRACKING);
                robot.head.setTracking(true);

            }
//            else if (gamepad1Buttons.wasJustPressed(GamepadButton.Y)) {
//                robot.head.runAnimation(HeadPresets.NodYes);
//            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.X)) {
//                robot.head.runAnimation(HeadPresets.ShakeNo);
//            }


            robot.update();

            FtcDashboardManager.update();
        }
    }
}