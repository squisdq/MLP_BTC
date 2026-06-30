package org.firstinspires.ftc.teamcode.teleop.comp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utils.Alliance;

@TeleOp(name = "BEX BLUE",group = "1111")
public class BexBlue extends BexRed {
    @Override
    public void setAlliance() {
        a = Alliance.BLUE;
    }
}
