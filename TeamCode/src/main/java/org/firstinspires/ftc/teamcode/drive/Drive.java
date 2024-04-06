package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Eyes;
import org.firstinspires.ftc.teamcode.util.Gyrobot;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;

@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() {
        Gyrobot gyrobot = new Gyrobot(hardwareMap);
        Eyes eyes = new Eyes(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();

            gyrobot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            gyrobot.update(telemetry);

            eyes.update(gamepad1.right_stick_x, gamepad1.right_stick_y);

            telemetry.update();
        }
    }
}