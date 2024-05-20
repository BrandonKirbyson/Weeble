package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Weeble;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;

@Config
@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    public static boolean DEBUG = false;
    public static boolean AnyTerrain = true;

    @Override
    public void runOpMode() {
        Weeble robot = new Weeble(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        waitForStart();

        boolean headMoving = false;

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();
            robot.update();


            robot.drive.update(telemetry);

            if (gamepad1Buttons.getButton(GamepadButton.LEFT_BUMPER)) {
                robot.drive.stopMotors();
            } else if (robot.drive.isBalanced()) {
                if (gamepad1.left_stick_y != 0) {
                    robot.drive.drive(-gamepad1.left_stick_y);
                } else {
                    robot.drive.idle();
                }
                if (gamepad1.left_stick_x != 0) {
                    robot.drive.turn(gamepad1.left_stick_x);
                }
            } else {
                if (gamepad1Buttons.getButton(GamepadButton.LEFT_STICK_BUTTON)) {
                    robot.drive.idle();
                } else {
                    robot.drive.stopMotors();
                }
            }
//
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
        }
    }
}