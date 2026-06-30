package org.firstinspires.ftc.teamcode.robot.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

public enum Alliance {
    BLUE(20, new Pose(2,141.2), new Pose(2,141.5)),//7.67,141.4)),
    RED(24, new Pose(142,141.5), new Pose(142,141.5));
    public final int id;
    public final Pose pose;
    public final Pose farPose;
    Alliance(int id, Pose pose,Pose farPose){
        this.id = id;
        this.pose = pose;
        this.farPose = farPose;
    }
}
