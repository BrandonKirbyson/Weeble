package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.drive.GyroDrive;
import org.firstinspires.ftc.teamcode.util.head.Head;

public class Weeble {
    public final GyroDrive drive;
    public final Head head;

    public Weeble(HardwareMap hardwareMap) {
        drive = new GyroDrive(hardwareMap);
        head = new Head(hardwareMap);
    }

    public void update() {
    }
}
