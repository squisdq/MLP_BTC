package org.firstinspires.ftc.teamcode.teleop.comp.bex;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utils.Alliance;

@Disabled
@TeleOp(name = "BEX BLUE",group = "1111")
public class BexBlue extends BexRed {
    @Override
    public void setAlliance() {
        a = Alliance.BLUE;
    }
}
