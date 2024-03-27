package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Gyrobot;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;

@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() {
        Gyrobot gyrobot = new Gyrobot(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);
        StatefulGamepad gamepad2Buttons = new StatefulGamepad(gamepad2);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();
            gamepad2Buttons.update();

            telemetry.update();
        }
    }
}