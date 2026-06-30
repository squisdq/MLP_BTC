package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;


@Config
public class Gate extends SubsystemBase {
    public Gate(){}
    public ServoEx g,ramp;
    public static double open_p = 0.58, close_p = 0.76;
    public static double open_ramp = 0.76, close_ramp = 0.58, mega_open_ramp = 0.515;
    public static double compression_time = 250;
    public boolean isOpen = false;
    public static double t = 0.76,t_ramp = 0.5;

    public static boolean activated = false;

    public void init(HardwareMap hw) {
        g = new ServoEx(hw,"g");
        ramp = new ServoEx(hw,"r");
        t_ramp = close_ramp;
        t = close_p;
    }

    public void activate(boolean a){
        activated = a;
    }

    public void setTarget(double pos,double ramp_p){
        t = pos;
        t_ramp = ramp_p;
    }

    private void setPosition(double pos,double ramp_p){
        g .set(pos);
        ramp.set(ramp_p);
    }

    public void open_gate(){
        isOpen = true;
        t = open_p;
        t_ramp = open_ramp;
    }

    public void open_gate(double time){
        isOpen = true;
        t = open_p;
        time = Math.max(0,Math.min(time, compression_time));
        t_ramp = open_ramp + (mega_open_ramp - open_ramp) * ( time / compression_time );
    }

    public void close_gate(){
        isOpen = false;
        t = close_p;
        t_ramp = close_ramp;
    }


    @Override
    public void periodic() {
        if(activated)
            setPosition(t,t_ramp);
    }

    public void toggle_gate(){
        isOpen = !isOpen;
        if(isOpen) open_gate();
        else close_gate();
    }

    public boolean isOpen(){
        return isOpen;
    }

    public InstantCommand open(){
        return new InstantCommand (this::open_gate);
    }

    public InstantCommand toggle(){
        return new InstantCommand (this::toggle_gate);
    }

    public InstantCommand close(){
        return new InstantCommand (this::close_gate);
    }


}
