package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.robot.utils.Alliance;

public class AlianceePaths {
    private int index = 0;
    private static final int PATH_COUNT = 9;
    Follower follower;

    public Pose p1_start = new Pose(31.205, 129.92, Math.toRadians(90)); //new Pose(18.701, 114.502, Math.toRadians(90));
    public Pose p1_end   = new Pose(58.376, 78.980,  Math.toRadians(170));

    public Pose p2_ctrl  = new Pose(57.192, 55.193,  Math.toRadians(170));
    public Pose p2_end   = new Pose(13.658, 58.989,  Math.toRadians(180));
    public Pose p3_ctrl   = new Pose(23.929, 59.13,  Math.toRadians(180));
    public Pose p3_end   = new Pose(18.599, 64.854,  Math.toRadians(180));
    public Pose p4_end   = new Pose(59.908, 76.001,  Math.toRadians(180));
    public Pose p5_end   = new Pose(17.881, 64.801,  Math.toRadians(-163));
    public Pose p7_ctrl   = new Pose(25.273, 57.748,  Math.toRadians(145));
    public Pose p7_end   = new Pose(10.241, 58.054,  Math.toRadians(143));

    public Pose p8_ctrl  = new Pose(50.428, 71.691,  Math.toRadians(145));
    public Pose p8_end   = new Pose(59.107, 78.040,  Math.toRadians(170));

    public Pose p9_end   = new Pose(18.497, 83.493,  Math.toRadians(170));
    public Pose p10_end  = new Pose(49.584, 114.677, Math.toRadians(170));
    public Pose p3rdRow_end =  new Pose(16.610, 39, Math.toRadians(180));
    public Pose p3rdRow_ctrl =  new Pose(41.77, 37, Math.toRadians(180));

    public Pose p11_end   =      new Pose(12.506, 40, Math.toRadians(180));
    public Pose p11_true_end_ctrl   = new Pose(11.134, 57.717, Math.toRadians(180));
    public Pose p11_true_end   = new Pose(8.0212, 13.597, Math.toRadians(180));
    public Pose p11_true_short_end   = new Pose(10.0212, 24.597, Math.toRadians(180));
    public Pose p12_end   =      new Pose(57.373, 21.356, Math.toRadians(180));

    public AlianceePaths(Alliance a, Follower f) {
        if (a.equals(Alliance.RED)) {
            p1_start = new Pose(112.3, 128.8, Math.toRadians(90));
            p1_end   = p1_end.mirror();
            p2_ctrl  = p2_ctrl.mirror();
            p2_end   = p2_end.mirror();
            p3_ctrl = p3_ctrl.mirror();
            p3_end   = p3_end.mirror();
            p4_end   = new Pose(61.208, 77.323,  Math.toRadians(180)).mirror();
            p5_end   = p5_end.mirror();
            p7_ctrl = p7_ctrl.mirror();
            p7_end   = new Pose(128.92, 59.604,  Math.toRadians(31.11));
            p8_ctrl  = p8_ctrl.mirror();
            p8_end   = new Pose(58.107, 77.040,  Math.toRadians(170)).mirror();
            p9_end   = p9_end.mirror();
            p10_end  = p10_end.mirror();
            p3rdRow_end = p3rdRow_end.mirror();
            p3rdRow_ctrl = p3rdRow_ctrl.mirror();
            p11_end =      new Pose(8.506, 40, Math.toRadians(180)).mirror();
            p11_true_end = new Pose(8.106, 14, Math.toRadians(180)).mirror();
            p12_end =      p12_end.mirror();
            p11_true_end_ctrl = p11_true_end_ctrl.mirror();
            p11_true_short_end = p11_true_short_end.mirror();
        }
        index = 0;
        follower = f;
    }

    public PathChain path1() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p1_start, p1_end))
                .setLinearHeadingInterpolation(p1_start.getHeading(), p1_end.getHeading())
                .build();
    }

    public PathChain path2() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(p1_end, p2_ctrl, p2_end))
                .setLinearHeadingInterpolation(p1_end.getHeading(), p2_end.getHeading())
                .build();
    }

    public PathChain path3() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(p2_end, p3_ctrl, p3_end))
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

    public PathChain path467() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p2_end, p4_end))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setConstraints( new PathConstraints(0.9, 100, 1, 1))
                .build();
    }

    public PathChain path2_467(){
        return follower.pathBuilder()
                .addPath(new BezierCurve(p1_end, p2_ctrl, p2_end))
                .setLinearHeadingInterpolation(p1_end.getHeading(), p2_end.getHeading())
                .addPath(new BezierLine(p2_end, p4_end))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public PathChain path5() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p4_end, p5_end))
                .setTangentHeadingInterpolation()
                .build();
    }

    public PathChain path57() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p4_end, p7_end))
                .setLinearHeadingInterpolation(p4_end.getHeading(), p7_end.getHeading())
                .build();
    }

    public PathChain path7() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(p5_end,p7_ctrl, p7_end))
                .setLinearHeadingInterpolation(p5_end.getHeading(), p7_end.getHeading())
                .build();
    }

    public PathChain path8() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(p7_end, p8_ctrl, p8_end))
                .setLinearHeadingInterpolation(p7_end.getHeading(), p8_end.getHeading())
                .build();
    }

    public PathChain path82() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p7_end, p8_end))
                .setLinearHeadingInterpolation(p7_end.getHeading(), p8_end.getHeading())
                .build();
    }

    public PathChain path867() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(p7_end,p8_ctrl, p10_end))
                .setTangentHeadingInterpolation()
                .setReversed()
//                .setLinearHeadingInterpolation(p7_end.getHeading(), p8_end.getHeading())
                .build();
    }

    public PathChain path9() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p8_end, p9_end))
                .setTangentHeadingInterpolation()
                .build();
    }

    public PathChain path10() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p9_end, p10_end))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public PathChain path67() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p9_end, p8_end))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public PathChain path9_67(){
        return follower.pathBuilder()
                .addPath(new BezierLine(p8_end, p9_end))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(p9_end, p8_end))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public PathChain path3rdRow() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(p4_end, p3rdRow_ctrl,p3rdRow_end))
                .setTangentHeadingInterpolation()
//                .setConstantHeadingInterpolation(p4_end.getHeading())
                .build();
    }

    public PathChain path3rdRowBackClose() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p3rdRow_end, p4_end))
                .setTangentHeadingInterpolation()
                .setConstraints( new PathConstraints(0.9, 100, 1, 1))
                .setReversed()
//                .setConstantHeadingInterpolation(p4_end.getHeading())
                .build();
    }

    public PathChain path3rdRowBackFar() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p3rdRow_end, p12_end))
                .setTangentHeadingInterpolation()
                .setConstraints( new PathConstraints(0.9, 100, 1, 1))
                .setReversed()
//                .setConstantHeadingInterpolation(p4_end.getHeading())
                .build();
    }

    public PathChain path11() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p12_end, p11_end))
                .setTangentHeadingInterpolation()
                .setConstraints( new PathConstraints(0.9, 100, 1, 1))
//                .setConstantHeadingInterpolation(p4_end.getHeading())
                .build();
    }

    public PathChain path12() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p11_end, p12_end))
                .setTangentHeadingInterpolation()
                .setConstraints( new PathConstraints(0.9, 100, 1, 1))
                .setReversed()
//                .setConstantHeadingInterpolation(p4_end.getHeading())
                .build();
    }

    public PathChain path4_11() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(p4_end,p11_true_end_ctrl, p11_true_end))
                .setTangentHeadingInterpolation()
                .setConstraints( new PathConstraints(0.99, 100, 0.3, 1))
                .build();
    }
    public PathChain path4_11_short() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(p4_end,p11_true_end_ctrl, p11_true_short_end))
                .setTangentHeadingInterpolation()
                .setConstraints( new PathConstraints(0.99, 100, 0.3, 1))
                .build();
    }

    public PathChain path4_11_true() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p11_end, p11_true_end))
                .setTangentHeadingInterpolation()
                .setConstraints( new PathConstraints(0.9, 100, 1, 1))
//                .setReversed()
//                .setConstantHeadingInterpolation(p4_end.getHeading())
                .build();
    }

    public PathChain path11_4_true() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p11_true_end, p4_end))
                .setTangentHeadingInterpolation()
                .setConstraints( new PathConstraints(0.97, 100, 1, 1))
                .setReversed()
//                .setConstantHeadingInterpolation(p4_end.getHeading())
                .build();
    }



    public PathChain path11_10_true() {
        return follower.pathBuilder()
                .addPath(new BezierLine(p11_true_end, p10_end))
                .setTangentHeadingInterpolation()
                .setConstraints( new PathConstraints(0.9, 100, 1, 1))
                .setReversed()
//                .setConstantHeadingInterpolation(p4_end.getHeading())
                .build();
    }
}