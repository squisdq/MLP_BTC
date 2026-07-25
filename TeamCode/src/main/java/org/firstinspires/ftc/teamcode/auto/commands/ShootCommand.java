package org.firstinspires.ftc.teamcode.auto.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class ShootCommand extends SequentialCommandGroup {
    boolean done;
    public ShootCommand(){
        done = false;
        Robot r = Robot.getInstance();
        addCommands(
//                new WaitCommand(100),
                r.intake.shootSpeedCommand(),
                new WaitCommand(400),
                r.intake.off(),
                r.gate.close(),
                r.intake.transfOn(),
                new InstantCommand(()->done = true)
        );
    }

    public ShootCommand(boolean isFar){
        done = false;
        Robot r = Robot.getInstance();
        addCommands(
                r.intake.shootSpeedFarCommand(),
                new WaitCommand(500),
                r.gate.close(),
                new InstantCommand(()->done = true)
        );
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
