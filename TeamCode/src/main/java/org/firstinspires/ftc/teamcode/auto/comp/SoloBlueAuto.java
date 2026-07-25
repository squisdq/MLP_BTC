package org.firstinspires.ftc.teamcode.auto.comp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
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


@Autonomous(name = "Solo Autie Blue", group = "SOLO")
public class SoloBlueAuto extends CommandOpMode {
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

        r.indicator.set(IndicatorLed.Color.PURPLE);

        schedule(
                new SequentialCommandGroup(
                        new WaitCommand(10),
                        r.intake.off(),
                        r.intake.in(),
                        new FollowPathCommand(r.drive.getFollower(), paths.path1())
                                .alongWith(new SequentialCommandGroup(new WaitCommand(20), farPreShoot())),
                        farShoot(),
                        new FollowPathCommand(r.drive.getFollower(), paths.path2()).alongWith(
                                r.intake.in()),

                        new FollowPathCommand(r.drive.getFollower(), paths.path467())
                                .alongWith(new SequentialCommandGroup(new WaitCommand(20), farPreShoot())),
                        farShoot(),

                        cycle(),

                        new FollowPathCommand(r.drive.getFollower(), paths.path9()).alongWith(
                                r.intake.in()),
                        new FollowPathCommand(r.drive.getFollower(), paths.path67()).alongWith(new SequentialCommandGroup(
                                new IntakeNorm(),new WaitCommand(20), farPreShoot())),
                        farShoot(),

                        cycle2(),

                        new FollowPathCommand(r.drive.getFollower(), paths.path3rdRow()).alongWith(
                                r.intake.in()),
                        new FollowPathCommand(r.drive.getFollower(), paths.path3rdRowBackClose(), false).alongWith(new SequentialCommandGroup(
                                new WaitCommand(20), farPreShoot())),
                        farShoot(),

                        cycleFar(),

                        new FollowPathCommand(r.drive.getFollower(), paths.path4_11(),false).alongWith(
                                r.intake.in()),

                        new FollowPathCommand(r.drive.getFollower(), paths.path11_10_true()).alongWith(new SequentialCommandGroup(
                                new IntakeNorm(),new WaitCommand(20), farPreShoot())),
                        farShoot()
                )
        );
    }

    public CommandBase cycle(){
        return new SequentialCommandGroup(
                new FollowPathCommand(r.drive.getFollower(), paths.path57(),false).alongWith(
                        r.intake.in()),
                new WaitCommand(700),
                new FollowPathCommand(r.drive.getFollower(), paths.path467()).alongWith(new SequentialCommandGroup(
                        new IntakeNorm(),new WaitCommand(20), farPreShoot())),
                farShoot());
    }

    public CommandBase cycle2(){
        return new SequentialCommandGroup(
                new FollowPathCommand(r.drive.getFollower(), paths.path57(),false).alongWith(
                        r.intake.in()),
                new WaitCommand(900),
                new FollowPathCommand(r.drive.getFollower(), paths.path467()).alongWith(new SequentialCommandGroup(
                        new IntakeNorm(),new WaitCommand(20), farPreShoot())),
                farShoot());
    }

    public CommandBase cycleFar(){
        return new SequentialCommandGroup(
                new FollowPathCommand(r.drive.getFollower(), paths.path4_11_short(),false).alongWith(
                        r.intake.in()),

//                new FollowPathCommand(r.drive.getFollower(), paths.path4_11_true(),false),
                new FollowPathCommand(r.drive.getFollower(), paths.path11_4_true()).alongWith(new SequentialCommandGroup(
                        new IntakeNorm(),new WaitCommand(20), farPreShoot())),
                farShoot());
    }

    public CommandBase farShoot(){
        return new SequentialCommandGroup(
                new ShootCommand(),
                new InstantCommand(()-> r.turret.unlock())
        );
    }

    public CommandBase farPreShoot(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> r.turret.lock()),
                new PreShooting()
        );
    }

    public void setAlliance(){
        a = Alliance.BLUE;
    }
    @Override
    public void run() {
        super.run();
        Pose futurePose = PoseController.getFuturePose(r.drive.getFollower(),0.4);
        r.turret.face(a.pose,
                futurePose);
        double dis = PoseController.getGoalDis(futurePose,a);
        if(PoseController.isInZone(r.drive.getPose())){
            r.intake.transferOn();
        }else{
            r.intake.transferOff();
        }
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

