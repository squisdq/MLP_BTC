package org.firstinspires.ftc.teamcode.auto.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class PreShooting  extends SequentialCommandGroup {
    boolean done;
    public PreShooting(){
        done = false;
        Robot r = Robot.getInstance();
        addCommands(
                r.intake.off(),
                r.shooter.on(),
                new WaitCommand(200),
                r.gate.open(),
                r.intake.transfOn(),
                new InstantCommand(()->done = true)
        );
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
