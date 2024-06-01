package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.usb.UsbSerialNumber;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "VisionTest", group = "tests")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        WebcamName camera = hardwareMap.get(WebcamName.class, new UsbSerialNumber("SN0001"));

        telemetry.addData("Camera", camera);
        telemetry.update();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(camera)
                .build();

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();


            telemetry.update();
        }
    }
}
