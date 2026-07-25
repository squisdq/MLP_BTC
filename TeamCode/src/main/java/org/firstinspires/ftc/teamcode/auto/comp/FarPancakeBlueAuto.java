package org.firstinspires.ftc.teamcode.auto.comp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;

import org.firstinspires.ftc.teamcode.auto.commands.IntakeNorm;
import org.firstinspires.ftc.teamcode.auto.commands.PreShooting;
import org.firstinspires.ftc.teamcode.auto.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.auto.paths.AlianceePaths;
import org.firstinspires.ftc.teamcode.auto.paths.FarAutoPath;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.leds.IndicatorLed;
import org.firstinspires.ftc.teamcode.robot.utils.Alliance;
import org.firstinspires.ftc.teamcode.robot.utils.PoseController;


@Autonomous(name = "Far Autie Blue", group = "FAR")
public class FarPancakeBlueAuto extends CommandOpMode {
    Robot r;
    Alliance a;
    FarAutoPath paths;
    @Override
    public void initialize() {
        super.reset();
        r = Robot.getInstance();
        setAlliance();
        r.drive.init(hardwareMap);
        paths = new FarAutoPath(a,r.drive.getFollower());
        r.init(hardwareMap,a,paths.p1_start);
        r.turnOn();

        r.indicator.set(IndicatorLed.Color.PURPLE);

        schedule(
                new SequentialCommandGroup(
                        r.intake.off(),
                        new WaitCommand(400),
                        farPreShoot(),
                        new WaitCommand(600),
                        farShoot(),
                        r.intake.in(),
                        new FollowPathCommand(r.drive.getFollower(), paths.path1()),
                        new WaitCommand(200),
                        new FollowPathCommand(r.drive.getFollower(), paths.path2())
                                .alongWith(new SequentialCommandGroup(new WaitCommand(100), farPreShoot())),
                        farShoot(),
                        r.intake.in(),
                        new FollowPathCommand(r.drive.getFollower(), paths.path3()),
                        new FollowPathCommand(r.drive.getFollower(), paths.path4())
                                .alongWith(new SequentialCommandGroup(new WaitCommand(100), farPreShoot())),
                        farShoot(),

                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),

                        new FollowPathCommand(r.drive.getFollower(), paths.path8()),

                        r.intake.off()
                )
        );
    }
    
    public CommandBase farShoot(){
        return new SequentialCommandGroup(
                new ShootCommand(true),
                new InstantCommand(()-> r.turret.unlock())
        );
    }

    public CommandBase farPreShoot(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> r.turret.lock()),
                new PreShooting()
        );
    }

    public CommandBase cycle(){
        return new SequentialCommandGroup(
                new FollowPathCommand(r.drive.getFollower(), paths.path5(),true).alongWith(
                        r.intake.in()),
                new FollowPathCommand(r.drive.getFollower(), paths.path6(),true),
                new FollowPathCommand(r.drive.getFollower(), paths.path7()).alongWith(new SequentialCommandGroup(
                        new IntakeNorm(),new WaitCommand(100), farPreShoot())),
                farShoot());
    }

    public void setAlliance(){
        a = Alliance.BLUE;
    }
    @Override
    public void run() {
        super.run();
        Pose futurePose = PoseController.getFuturePose(r.drive.getFollower(),0.4);
        r.turret.face(a.farPose,
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

