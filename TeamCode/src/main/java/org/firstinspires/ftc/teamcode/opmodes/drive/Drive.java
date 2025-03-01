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
@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    protected DriveType getDriveType() {
        return DriveType.SMOOTH;
    }

    @Override
    public void runOpMode() {
        Weeble robot = new Weeble(hardwareMap, getDriveType());

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);
        StatefulGamepad gamepad2Buttons = new StatefulGamepad(gamepad2);

        waitForStart();

//        robot.vision.setMode(VisionMode.DISABLED);

        robot.head.reset();

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
//                robot.drive.setTargetPos();
//                robot.drive.setTargetAngle(PIDConstants.TargetAngle);
                robot.drive.brake();
            }

//            if (gamepad2Buttons.getButton(GamepadButton.DPAD_RIGHT)) {
//                robot.drive.emergencyStop();
//            }
//
//            if (gamepad2Buttons.wasJustPressed(GamepadButton.X) && gamepad2Buttons.getButton(GamepadButton.RIGHT_BUMPER)) {
//                robot.drive.setDriveType(DriveType.NONE);
//            } else if (gamepad2Buttons.wasJustPressed(GamepadButton.Y)) {
//                robot.drive.setDriveType(DriveType.OFFROAD);
//            } else if (gamepad2Buttons.wasJustPressed(GamepadButton.A)) {
//                robot.drive.setDriveType(DriveType.SMOOTH);
//            }

//            if (gamepad1Buttons.wasJustPressed(GamepadButton.LEFT_STICK_BUTTON) && robot.drive.getState() == DriveState.STOPPED) {
//                robot.uprightWithArms();
//            }

//            if (gamepad1Buttons.wasJustPressed(GamepadButton.RIGHT_STICK_BUTTON)) {
//            }

//            if (gamepad2Buttons.wasJustPressed(GamepadButton.Y)) {
//                robot.vision.setRecording(true);
//            } else if (gamepad2Buttons.wasJustPressed(GamepadButton.X)) {
//                robot.vision.setRecording(false);
//            }
//            FtcDashboardManager.addData("RECORDING", robot.vision.isRecording());
            FtcDashboardManager.addData("Drive Type", robot.drive.getDriveType());

//            if (gamepad1.right_stick_y != 0 || gamepad2.right_stick_x != 0) {
            if (!gamepad1Buttons.getButton(GamepadButton.DPAD_RIGHT) && !gamepad1Buttons.getButton(GamepadButton.X)) {
                //noinspection SuspiciousNameCombination
                robot.head.manualControl(gamepad1.right_stick_y, gamepad1.right_stick_x);
            } else {
                robot.head.manualControlAdjust(gamepad1.right_stick_y, gamepad1.right_stick_x);
            }

            if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                robot.head.reset();
                robot.head.manualControl(0, 0);
            }


//            }

            if (robot.drive.getTurnVelocity() == 0) {
                robot.head.setEyes(0.5 + (-gamepad1.left_trigger + gamepad1.right_trigger) / 2);
            }
////
//            if (robot.drive.getDriveType() == DriveType.NONE) {
//                if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
//                    robot.vision.setMode(VisionMode.DISABLED);
//                    robot.head.reset();
//                    robot.head.setEyebrows(HeadConstants.eyebrowsNeutral);
//                }
//                if (gamepad1Buttons.wasJustPressed(GamepadButton.B)) {
//                    robot.vision.setMode(VisionMode.ENABLED);
//                    robot.head.setTracking(false);
//                }
//                if (gamepad1Buttons.wasJustPressed(GamepadButton.X)) {
//                    robot.vision.setMode(VisionMode.BLOB_TRACKING);
//                    robot.head.setTracking(true);
//                }
//            }

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

//            if (gamepad1Buttons.wasJustPressed(GamepadButton.Y)) {
//                robot.head.initHoldHead();
//            }
//            if (gamepad1Buttons.getButton(GamepadButton.Y)) {
//                robot.head.holdHead();
//            }

            if (gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) {
                robot.arms.setLeftArmPosition(gamepad2.left_stick_y);
                robot.arms.setRightArmPosition(gamepad2.right_stick_y);
            }

            FtcDashboardManager.update();
        }
    }
}