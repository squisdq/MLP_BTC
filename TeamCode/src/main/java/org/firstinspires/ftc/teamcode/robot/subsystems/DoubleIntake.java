package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
public class DoubleIntake extends SubsystemBase {
    public DoubleIntake(){}

    MotorEx mr,ml;
    ServoEx pto;

    public static double transferOnPos = 0.08;
    public static double transferOffPos = 0.15;


    public static double in = 1;
    public static double out = -1;
    public static double off = 0;
    public static double shoot = 1;
    public static double slowShoot = 1;

    public static double inSpeed = 2100;
    public static double shootSpeed = 2100;
    public static double shootSpeedClose = 2100;

    public static double kP = 0.0005, kF = 0.000527;
    public void init(HardwareMap hw){
        mr = new MotorEx(hw,"i1");
        ml = new MotorEx(hw,"i2");
        ml.setInverted(true);

        pto = new ServoEx(hw, "pto");
    }
    public void setPower(double power){
        mr.set(power);
        ml.set(power);
    }
    public double getVelocity(){
        return mr.getVelocity();
    }
    public void setSpeed(double speed){
        double error =  speed - getVelocity();
        setPower(speed * kF + error * kP);
    }
    public void setServoPos(double pos){
        pto.set(pos);
    }
    public void transferOn(){
        setServoPos(transferOnPos);
    }
    public void transferOff(){
        setServoPos(transferOffPos);
    }
    public void speedIn(){
        setSpeed(inSpeed);
        transferOff();
    }
    public void speedShoot(){
        setSpeed(shootSpeed);
        transferOn();
    }
    public void speedShootClose(){
        setSpeed(shootSpeedClose);
        transferOn();
    }
    public void spinIn(){
        setPower(in);
        transferOff();
    }
    public void spinOut(){
        setPower(out);
        transferOn();
    }
    public void spinOff(){
        setPower(off);
        transferOff();
    }
    public void spinShoot(){
        setPower(shoot);
        transferOn();
    }
    public void spinSlowShoot(){
        setPower(slowShoot);
        transferOn();
    }
    public InstantCommand transfOn(){return new InstantCommand(this::transferOn);}
    public InstantCommand shoot(){return new InstantCommand(this::spinShoot);}
    public InstantCommand slowShoot(){return new InstantCommand(this::spinSlowShoot);}
    public InstantCommand in(){
        return new InstantCommand(this::spinIn);
    }

    public InstantCommand shootSpeedCommand(){
        return new InstantCommand(this::speedShootClose);
    }

    public InstantCommand shootSpeedFarCommand(){
        return new InstantCommand(this::speedShoot);
    }
    public InstantCommand out(){
        return new InstantCommand(this::spinOut);
    }
    public InstantCommand off(){
        return new InstantCommand(this::spinOff);
    }
}
