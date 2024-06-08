package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;

@TeleOp(name = "ServosTest", group = "tests")
public class ServosTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo head = hardwareMap.get(Servo.class, "head");
        Servo rightArm = hardwareMap.get(Servo.class, "arm0");
        Servo leftArm = hardwareMap.get(Servo.class, "arm1");
        Servo neck = hardwareMap.get(Servo.class, "neck");
        Servo eyes = hardwareMap.get(Servo.class, "eyes");
        Servo eyebrows = hardwareMap.get(Servo.class, "eyebrows");

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);
        waitForStart();

        Servo currentServo = head;

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();

            if (gamepad1Buttons.wasJustPressed(GamepadButton.Y)) {
                currentServo = eyebrows;
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                currentServo = eyes;
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.X)) {
                currentServo = neck;
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.B)) {
                currentServo = head;
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.RIGHT_BUMPER)) {
                currentServo = rightArm;
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.LEFT_BUMPER)) {
                currentServo = leftArm;
            }

            currentServo.setPosition(gamepad1.left_stick_y / 2 + 0.5);

            telemetry.update();
        }
    }
}