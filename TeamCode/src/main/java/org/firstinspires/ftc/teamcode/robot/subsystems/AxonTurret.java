package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class AxonTurret extends SubsystemBase {
    public AxonTurret(){}
    private ServoEx axon1,axon2;
    public static double rpt = 5.511566;
    public static double t = 0;

    public static double offset = 0;
    public static double oneServoOffset = 0;
    public static double rAM = -0.14;

    public static boolean on = true;

    public static double limit = Math.PI/6;

    public void init(HardwareMap hw) {
        axon1 = new ServoEx(hw,"t1");
        axon2 = new ServoEx(hw,"t2");
    }

    private void setTurretTarget(double ticks) {
        t = ticks + 0.5 - offset;
    }

    public double getTurretTarget() {
        return t;
    }
    public double getPos(){return 0;}

    public void setPos(double pos){
        axon1.set(pos - oneServoOffset);
        axon2.set(pos + oneServoOffset);
    }


    @Override
    public void periodic() {
        if (on) {
            setPos(getTurretTarget());
        }
    }

    public void lock(){
        oneServoOffset = 0.035;
    }

    public void unlock(){
        oneServoOffset = 0.00;
    }

    public void face(Pose targetPose, Pose robotPose) {
        face(targetPose, robotPose, 0);
    }

    public void face(Pose targetPose, Pose robotPose, double angleVel) {
        double y =  targetPose.getY() - robotPose.getY();

        double x = targetPose.getX() - robotPose.getX();

        if(Math.abs(angleVel)<2.5){
            angleVel = 0;
        }

        double angleToTargetFromCenter = Math.atan2(y, x) + angleVel * rAM  ;
        double robotAngleDiff = normalizeAngle(angleToTargetFromCenter - robotPose.getHeading());
        setYaw(robotAngleDiff);
    }

    public void face(Pose targetPose, Pose robotPose, double angleVel, double yawOffsetRadians) {
        double y =  targetPose.getY() - robotPose.getY();

        double x = targetPose.getX() - robotPose.getX();

        if(Math.abs(angleVel)<2.5){
            angleVel = 0;
        }

        double angleToTargetFromCenter = Math.atan2(y, x) + angleVel * rAM  ;
        double robotAngleDiff = normalizeAngle(angleToTargetFromCenter - robotPose.getHeading() + yawOffsetRadians);
        setYaw(robotAngleDiff);
    }

    public InstantCommand faceCommand(Pose targetPose, Pose robotPose) {
        return new InstantCommand(() -> face(targetPose,robotPose));
    }
    public void on() {
        on = true;
    }

    public void off() {
        on = false;
    }

    public double getYaw() {
        return normalizeAngle(getPos() * rpt);
    }

    public void setYaw(double radians) {
        radians = normalizeAngle(radians);
        setTurretTarget(normalizeAngle(radians - Math.PI)/rpt);
    }

    public void addYaw(double radians) {
        setYaw(getYaw() + radians);
    }

    public InstantCommand set(double radians) {
        return new InstantCommand(() -> setYaw(radians));
    }

    public InstantCommand add(double radians) {
        return new InstantCommand(() -> setYaw(getYaw() + radians));
    }

    public static double normalizeAngle(double angleRadians) {
//        double angle = angleRadians % (Math.PI * 2D);
//        if (angle <= -Math.PI) angle += Math.PI * 2D;
//        if (angle > Math.PI) angle -= Math.PI * 2D;
//        angle = rangeAngle(angle);
        return AngleUnit.normalizeRadians(angleRadians);
    }

    public static double rangeAngle(double radians){
//        if (radians > -limit && radians < limit) {
//            radians = (radians >= 0) ? limit : -limit;
//        }
        return radians;
    }
}
