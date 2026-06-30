package org.firstinspires.ftc.teamcode.auto.comp;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.commands.IntakeNorm;
import org.firstinspires.ftc.teamcode.auto.commands.PreShooting;
import org.firstinspires.ftc.teamcode.auto.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.auto.paths.AlianceePaths;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.leds.IndicatorLed;
import org.firstinspires.ftc.teamcode.robot.utils.Alliance;
import org.firstinspires.ftc.teamcode.robot.utils.PoseController;

@Autonomous(name = "Autie Blue", group = "CLOSE")
public class PancakeBlueAuto extends CommandOpMode {
    Robot r;
    Alliance a;
    AlianceePaths paths;
    @Override
    public void initialize() {
        super.reset();
        r = Robot.getInstance();
        setAlliance();
        r.drive.init(hardwareMap);
        paths = new AlianceePaths(a,r.drive.getFollower());
        r.init(hardwareMap,a,paths.p1_start);
        r.turnOn();
        r.intake.transferOn();

        r.prism.setAllianceColor(a);

        r.indicator.set(IndicatorLed.Color.PURPLE);

        schedule(
                new SequentialCommandGroup(
                        new WaitCommand(400),
                        r.intake.off(),
                        r.intake.in(),
                        new FollowPathCommand(r.drive.getFollower(), paths.path1())
                                .alongWith(new SequentialCommandGroup(new WaitCommand(300), new PreShooting())),
                        new ShootCommand(),
                        new FollowPathCommand(r.drive.getFollower(), paths.path2()).alongWith(
                                r.intake.in()),

                        new FollowPathCommand(r.drive.getFollower(), paths.path3()),
                        new FollowPathCommand(r.drive.getFollower(), paths.path4())
                                .alongWith(new SequentialCommandGroup(new WaitCommand(300), new PreShooting())),
                        new ShootCommand(),

                        new WaitCommand(400),
                        cycle(),
                        new WaitCommand(300),
                        cycle(),
                        new WaitCommand(300),
                        cycle(),
                        new WaitCommand(300),
                        cycle(),


                        new FollowPathCommand(r.drive.getFollower(), paths.path9()).alongWith(
                                r.intake.in()),
                        new FollowPathCommand(r.drive.getFollower(), paths.path10()).alongWith(new SequentialCommandGroup(
                                new IntakeNorm(),new WaitCommand(100), new PreShooting())),
                        new ShootCommand(),
                        r.intake.off()
                )
        );
    }

    public CommandBase cycle(){
        return new SequentialCommandGroup(
                new FollowPathCommand(r.drive.getFollower(), paths.path57(),true).alongWith(
                        r.intake.in()),
                new WaitCommand(850),//1200
                new FollowPathCommand(r.drive.getFollower(), paths.path8()).alongWith(new SequentialCommandGroup(
                        new IntakeNorm(),new WaitCommand(100), new PreShooting())),
                new ShootCommand());
    }

    public void setAlliance(){
        a = Alliance.BLUE;
    }
    @Override
    public void run() {
        super.run();
        r.intake.transfOn();
        Pose futurePose = PoseController.getFuturePose(r.drive.getFollower(), 0.5);
        r.turret.face(a.pose,
               futurePose);
        double dis = PoseController.getGoalDis(futurePose,a);
        r.shooter.setDis(dis);
        r.hood.setDis(dis);
        r.saveEnd();
        r.periodic();
    }
    @Override public void end() {
        r.autoBeen();
        r.saveEnd();
    }
}
