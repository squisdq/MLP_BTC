package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.robot.utils.Alliance;

public class FarAutoPath{
    Follower follower;

    public Pose p1_start = new Pose(58.798, 7.3708, Math.toRadians(180));
    public Pose p1_end   = new Pose(16.106, 9, Math.toRadians(180));
    public Pose p1_back   = new Pose(16.106, 9, Math.toRadians(180));

    public Pose p2_end   = new Pose(60.373, 18.356, Math.toRadians(180));

    public Pose p3_end   = new Pose(18.617, 37.433, Math.toRadians(180));
    public Pose p3_ctrl   = new Pose(29.00, 36.60, Math.toRadians(180));

    public Pose p4_end   = new Pose(60.760, 18.114, Math.toRadians(180));

    public Pose p5_end   = new Pose(18.800, 9.5, Math.toRadians(180));

    public Pose p6_ctrl   = new Pose(18, 12, Math.toRadians(180));

    public Pose p6_end   = new Pose(18.800, 28.528, Math.toRadians(127));

    public Pose p7_end   = new Pose(60.313, 21.298, Math.toRadians(180));

    public Pose p8_end   = new Pose(37.573, 21.276, Math.toRadians(180));

    public FarAutoPath(Alliance a, Follower f) {
        if (a.equals(Alliance.RED)) {
            p1_start = p1_start.mirror();
            p1_end   = p1_end.mirror();
            p1_back = p1_back.mirror();
            p2_end   = p2_end.mirror();
            p3_end   = p3_end.mirror();
            p3_ctrl = p3_ctrl.mirror();
            p4_end   = p4_end.mirror();
            p5_end   = p5_end.mirror();
            p6_end   = p6_end.mirror();
            p6_ctrl = p6_ctrl.mirror();
            p7_end   = p7_end.mirror();
            p8_end   = p8_end.mirror();
        }
        follower = f;
    }

    public PathChain path1() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p1_start, p1_end))
                .setLinearHeadingInterpolation(p1_start.getHeading(), p1_start.getHeading())
                .build();
    }
    public PathChain path1_back() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p1_end, p1_back))
                .setLinearHeadingInterpolation(p1_start.getHeading(), p1_start.getHeading())
                .build();
    }

    public PathChain path1_forward() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p1_back, p1_end))
                .setLinearHeadingInterpolation(p1_start.getHeading(), p1_start.getHeading())
                .build();
    }

    public PathChain path2() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p1_end, p2_end))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public PathChain path3() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(p2_end, p3_ctrl,p3_end))
                .setConstantHeadingInterpolation(p3_end.getHeading())
                .build();
    }

    public PathChain path4() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p3_end, p4_end))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public PathChain path5() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p4_end, p5_end))
                .setConstantHeadingInterpolation(p4_end.getHeading())
                .build();
    }

    public PathChain path6() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p5_end, p6_end))
                .setConstantHeadingInterpolation(p6_end.getHeading())
                .build();
    }

    public PathChain path7() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p6_end, p7_end))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public PathChain path8() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p7_end, p8_end))
                .setTangentHeadingInterpolation()
                .build();
    }
}