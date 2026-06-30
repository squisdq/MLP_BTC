package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.utils.DennyLUT;


@Config
public class Hood extends SubsystemBase {
    public Hood(){}
    public ServoEx h;

    public static double t = 0.5;
    public static double min = 0.2;
    public static double max = 1;
    public static double compM = 0.9;
    public static double compShoo = 800;
    public static boolean isCompMode = false;

    DennyLUT lut;

    public static boolean activated = false;

    public void init(HardwareMap hw) {
        h = new ServoEx(hw,"h");

        lut = new DennyLUT();


        lut.add(74.5,0.7);//0.7
        lut.add(99.6,0.6);//0.6
        lut.add(137.9,0.4);//0.4
        lut.add(144, 0.1);

//        lut.add(49.2,  1);
//        lut.add(54.2,1);
//        lut.add(75.8,  0.52);
//        lut.add(90.4,  0.40);
//
//        lut.add(102.6, 0.37);
//        lut.add(135.6, 0.39);
//        lut.add(150.6, 0.31);
    }

    public double getTarget() {
        return t;
    }
    public void activate(boolean a){
        activated = a;
    }

    public double getHood() {
        return h.getRawPosition();
    }

    @Override
    public void periodic() {
        if(activated)
            h.set(t);
    }

    public void setPos(double pos){
        t = pos;
    }

    public void setTarget(double pos){
        t = pos;
        if(t > max) t = max;
        else if(t<min) t = min;
    }

    public void setTarget(double pos, double x){
        t = pos;

        if(isCompMode){
            Shooter s = Robot.getInstance().shooter;
            double predictedVelocity = s.getVelocity() + s.getAcceleration() * 1.6 ;
            double error = Math.abs(predictedVelocity - s.lut.get(x));
            if(error < 30){
                error = 0;
            }
            double m = (error/compShoo) * compM;
            if(m > compM) m = compM;
            t += m;
        }

        if(t > max) t = max;
        else if(t<min) t = min;
    }

    public InstantCommand set(double v){return new InstantCommand(()->setTarget(v)); }

    public void setCompMode(boolean a){
        isCompMode = a;
    }

    public void setDis(double x){
        setTarget(lut.get(x),x);
    }
}
