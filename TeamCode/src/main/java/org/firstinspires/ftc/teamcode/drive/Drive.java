package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Gyrobot;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;

@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() {
        Gyrobot robot = new Gyrobot(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();

            robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            robot.update(telemetry);

            if (!gamepad1Buttons.getButton(GamepadButton.Y)) {
                robot.head.setManualPosition(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.right_trigger - gamepad1.left_trigger);
            } else {
                robot.head.holdPosition();
            }

            telemetry.update();
        }
    }
}