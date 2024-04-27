package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Gyrobot;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;

@Config
@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    public static boolean DEBUG = false;
    public static boolean AnyTerrain = true;

    @Override
    public void runOpMode() {
        Gyrobot robot = new Gyrobot(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        waitForStart();

        boolean heangdMoving = false;

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();
            robot.update(telemetry);

            if (gamepad1Buttons.getButton(GamepadButton.LEFT_BUMPER)) {
                robot.stopMotors();
            } else if (robot.isBalanced()) {
                if (gamepad1.left_stick_y != 0) {
                    robot.drive(-gamepad1.left_stick_y);
                } else {
                    robot.idle();
                }
                if (gamepad1.left_stick_x != 0) {
                    robot.turn(gamepad1.left_stick_x);
                }
            } else {
                if (gamepad1Buttons.getButton(GamepadButton.LEFT_STICK_BUTTON)) {
                    robot.idle();
                } else {
                    robot.stopMotors();
                }
            }

            if (gamepad1Buttons.getButton(GamepadButton.Y)) {
                robot.head.holdPosition();
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                robot.head.reset();
            } else if (gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0) {
                heangdMoving = true;
                robot.head.setManualPosition(gamepad1.right_stick_y, gamepad1.right_stick_x);
            } else if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                robot.head.adjustSetPosition(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right);
            } else if (heangdMoving) {
                robot.head.reset();
                heangdMoving = false;
            }

//            if (gamepad1Buttons.wasJustPressed(GamepadButton.B)) {
//                robot.head.resetEyes();
//            } else if (gamepad1.right_trigger != 0 || gamepad1.left_trigger != 0) {
//            }
            robot.head.setEyes(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }
}