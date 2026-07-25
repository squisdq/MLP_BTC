package org.firstinspires.ftc.teamcode.robot;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.leds.IndicatorLed;
import org.firstinspires.ftc.teamcode.robot.subsystems.AxonTurret;
import org.firstinspires.ftc.teamcode.robot.subsystems.DoubleIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.*;
import org.firstinspires.ftc.teamcode.robot.utils.Alliance;
import org.firstinspires.ftc.teamcode.robot.utils.VoltageController;

public class Robot {
    private static final Robot INSTANCE = new Robot();
    private Robot(){
        drive = new Drive();
        gate = new Gate();
        intake = new DoubleIntake();
        shooter = new Shooter();
        hood = new Hood();
        turret = new AxonTurret();
        voltage = new VoltageController();
        indicator = new IndicatorLed();
    }
    public static Robot getInstance(){return INSTANCE;}
    private Alliance a;
    public final Drive drive;
    public final Gate gate;
    public final DoubleIntake intake;
    public final Shooter shooter;
    public final Hood hood;
    public final AxonTurret turret;
    public final VoltageController voltage;
    public final IndicatorLed indicator;

    public static Pose defaultPose = new Pose(72, 72, Math.toRadians(90));
    private static boolean isAutoBeen = false;

    public void turnOn(){
        gate.activate(true);
        hood.activate(true);
        shooter.turnOn();
        turret.on();
        indicator.on();
//        kickstand.activate(true);
    }

    public void turnOff(){
        gate.activate(false);
        hood.activate(false);
        shooter.turnOff();
        turret.off();
    }


    public void init(HardwareMap hw, Alliance a){
        init(hw,a,defaultPose);
    }

    public void init(HardwareMap hw, Alliance a, Pose startPose){
        this.a = a;
        drive.init(hw,a,startPose);
        gate.init(hw);
        intake.init(hw);
        shooter.init(hw);
        hood.init(hw);
        turret.init(hw);
        voltage.init(hw);
        indicator.init(hw);
    }

    public void periodic(){
        drive.periodic();
        gate.periodic();
        intake.periodic();
        shooter.periodic();
        hood.periodic();
        turret.periodic();
        indicator.periodic();
    }

    public void saveEnd() {
        if(drive.getPose().getX() > 1)
            defaultPose = drive.getPose();
    }

    public Alliance getAlliance(){return a;}

    public void autoBeen(){
        isAutoBeen = true;
    }

    public void autoNotBeen(){
        isAutoBeen = false;
    }

    public boolean isAutoBeen(){return isAutoBeen;}


}
