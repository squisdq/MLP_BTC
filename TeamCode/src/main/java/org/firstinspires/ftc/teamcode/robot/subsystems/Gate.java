package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;


@Config
public class Gate extends SubsystemBase {
    public Gate(){}
    public ServoEx g;
    public static double open_p = 0.6, close_p = 0.35;
    public boolean isOpen = false;
    public static double t = 0.76;

    public static boolean activated = false;

    public void init(HardwareMap hw) {
        g = new ServoEx(hw,"g");
        t = close_p;
    }

    public void activate(boolean a){
        activated = a;
    }

    public void setTarget(double pos){
        t = pos;
    }

    private void setPosition(double pos){
        g .set(pos);
    }

    public void open_gate(){
        isOpen = true;
        t = open_p;
    }
    public void close_gate(){
        isOpen = false;
        t = close_p;
    }


    @Override
    public void periodic() {
        if(activated)
            setPosition(t);
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
