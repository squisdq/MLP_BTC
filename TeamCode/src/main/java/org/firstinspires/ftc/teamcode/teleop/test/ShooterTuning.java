package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

@TeleOp(group = "test")
@Config
//@Disabled
public class ShooterTuning extends OpMode {
    Shooter s;
    TelemetryPacket packet;
    FtcDashboard dash;
    @Override
    public void init() {
        s = new Shooter();
        s.init(hardwareMap);

        packet = new TelemetryPacket();
        dash = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        s.periodic();

        packet.put("s vel",s.getVelocity());
        packet.put("s target vel",s.getTarget());

        dash.sendTelemetryPacket(packet);
    }
}
