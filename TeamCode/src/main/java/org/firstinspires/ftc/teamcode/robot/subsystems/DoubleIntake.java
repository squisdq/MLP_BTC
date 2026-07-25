package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Config
public class DoubleIntake extends SubsystemBase {
    public DoubleIntake(){}

    MotorEx im, tm;

    public static double in = 1;
    public static double out = -1;
    public static double off = 0;
    public static double shoot = 1;
    public static double slowShoot = 1;

    boolean enableTransfer = false;

    public void init(HardwareMap hw){
        im = new MotorEx(hw,"in");//вращает интейк и вертель на мангале
        tm = new MotorEx(hw,"tr");//ращает трансфер (i hope)
        tm.setInverted(true);
    }
    public void setPower(double power){
        im.set(power);
        if(enableTransfer)
            tm.set(power);
        else
            tm.set(0);
    }
    public double getVelocity(){
        return im.getVelocity();
    }
    public void transferOn(){
        enableTransfer = true;
    }
    public void transferOff(){
        enableTransfer = false;
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
//        transferOff();
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
    public InstantCommand in(){return new InstantCommand(this::spinIn);
    }

    public InstantCommand shootSpeedCommand(){
        return new InstantCommand(this::spinShoot);
    }

    public InstantCommand shootSpeedFarCommand(){
        return new InstantCommand(this::spinSlowShoot);
    }
    public InstantCommand out(){
        return new InstantCommand(this::spinOut);
    }
    public InstantCommand off(){
        return new InstantCommand(this::spinOff);
    }
}
