package org.firstinspires.ftc.teamcode.drive.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Eyes;

@TeleOp(name = "EyesTest", group = "tests")
public class EyesTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Eyes eyes = new Eyes(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            eyes.update(gamepad1.left_stick_x, gamepad1.left_stick_y);
        }
    }
}