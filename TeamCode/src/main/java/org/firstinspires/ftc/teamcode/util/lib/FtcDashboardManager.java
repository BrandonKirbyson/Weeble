package org.firstinspires.ftc.teamcode.util.lib;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class FtcDashboardManager {
    private static TelemetryPacket packet = new TelemetryPacket();
    private static final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static void addData(String caption, Object value) {
        packet.put(caption, value);
    }

    public static TelemetryPacket getPacket() {
        return packet;
    }

    public static void update() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
