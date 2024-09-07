package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.drive.DriveType;

@TeleOp(name = "Offroad Drive")
public class OffroadDrive extends Drive {
    @Override
    protected DriveType getDriveType() {
        return DriveType.OFFROAD;
    }
}
