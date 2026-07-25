package org.firstinspires.ftc.teamcode.robot.leds;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.utils.Alliance;

@Config
public class IndicatorLed extends SubsystemBase {
    public static boolean activated = false;
    Servo indicator, masturbator;
    public static double color = 0;

    public static double min = 0.28;
    public static double max = 0.72;
    public static double animTime = 1;
    public static boolean animate = false;
    public enum Color{
        RED(0.28),
        ORANGE(0.3),
        YELLOW(0.35),
        GREEN(0.49),
        BLUE(0.61),
        PURPLE(0.72),
        WHITE(1);

        final double color;
        Color(double color){
            this.color = color;
        }
    }
    ElapsedTime timer;
    public void init(HardwareMap hw){
        indicator = hw.get(Servo.class,"rgbi");
        masturbator = hw.get(Servo.class, "lgbt");

        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void periodic() {
        if(activated){
            if(animate){
                double temp = (timer.milliseconds()%(2* animTime));
                temp = temp>animTime?(2* animTime - temp):temp;
                color = min + (max - min) * temp / animTime;
            }
            indicator.setPosition(color);
            masturbator.setPosition(color);
        }
    }

    public void set(double color){
        IndicatorLed.color = color;
    }
    public void set(Color color){
        IndicatorLed.color = color.color;
    }

    public void set(Alliance alliance){
        IndicatorLed.color = alliance.equals(Alliance.BLUE)?Color.BLUE.color:Color.RED.color;
    }

    public void on(){
        activated = true;
    }
    public void off(){
        activated = false;
    }
    public void turnAnimOn(){
        animate = true;
    }
    public void turnAnimOff(){
        animate = false;
    }
}
