package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.arms.ArmPosition;
import org.firstinspires.ftc.teamcode.util.arms.Arms;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;

@TeleOp(name = "ArmsTest", group = "tests")
public class ArmsTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Arms arms = new Arms(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();

            if (gamepad1Buttons.wasJustPressed(GamepadButton.DPAD_UP)) {
                arms.setArmPosition(ArmPosition.Up);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.DPAD_DOWN)) {
                arms.setArmPosition(ArmPosition.Down);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.DPAD_RIGHT)) {
                arms.setArmPosition(ArmPosition.Forward);
            }

            telemetry.update();
        }
    }
}