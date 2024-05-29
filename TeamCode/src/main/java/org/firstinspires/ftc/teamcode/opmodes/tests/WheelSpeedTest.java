package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;

@TeleOp(name = "WheelSpeedTest", group = "tests")
public class WheelSpeedTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor left = hardwareMap.get(DcMotor.class, "left");
        DcMotor right = hardwareMap.get(DcMotor.class, "right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        waitForStart();

        double prevLeftTicks = left.getCurrentPosition();
        double prevRightTicks = right.getCurrentPosition();

        double prevTime = System.currentTimeMillis();

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();

            left.setPower(gamepad1.left_stick_y);
            right.setPower(gamepad1.left_stick_y);

            double time = System.currentTimeMillis();
            double deltaTime = time - prevTime;

            double leftTicks = left.getCurrentPosition();
            double rightTicks = right.getCurrentPosition();


            double leftSpeed = (leftTicks - prevLeftTicks) / (deltaTime);
            double rightSpeed = (rightTicks - prevRightTicks) / (deltaTime);

            telemetry.addData("Loop Time", deltaTime);

            telemetry.addLine("---");

            telemetry.addData("Drive Power", gamepad1.left_stick_y);

            telemetry.addLine("");

            telemetry.addData("Left Speed", leftSpeed);
            telemetry.addData("Right Speed", rightSpeed);

            telemetry.addLine("");

            double avgSpeed = (leftSpeed + rightSpeed) / 2;
            telemetry.addData("Average Speed", avgSpeed);

            prevLeftTicks = leftTicks;
            prevRightTicks = rightTicks;

            prevTime = time;

            telemetry.update();
        }
    }
}