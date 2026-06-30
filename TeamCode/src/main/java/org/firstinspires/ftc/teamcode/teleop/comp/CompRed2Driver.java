package org.firstinspires.ftc.teamcode.teleop.comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.robot.leds.IndicatorLed;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.utils.Alliance;
import org.firstinspires.ftc.teamcode.robot.utils.PoseController;
import org.firstinspires.ftc.teamcode.teleop.DennyOpMode;


//@TeleOp(name = "REEEEED",group = "111")
public class CompRed2Driver extends DennyOpMode {
    TelemetryPacket packet;
    FtcDashboard dash;
    boolean holdTurret = false;
    ElapsedTime gating;
    boolean compress = false;
    @Override
    public void onInit() {
        super.onInit();
        packet = new TelemetryPacket();
        dash = FtcDashboard.getInstance();
        gating = new ElapsedTime();
    }

    @Override
    public void setAlliance() {
        a = Alliance.RED;
    }

    @Override
    public void onStart() {
        r.turnOn();
        r.drive.startDrive();
        r.shooter.enableVoltageCompensation(true);


        r.prism.setAllianceColor(a);

        r.prism.update();
        r.indicator.set(IndicatorLed.Color.RED);
    }

    @Override
    public void onUpdate() {

        if(base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            r.drive.teleToggleCentric();
        }
        r.drive.drive(base);

        Pose futurePose = PoseController.getFuturePose( r.drive.getFollower());


        if(helper.wasJustPressed(GamepadKeys.Button.B)){
            holdTurret = !holdTurret;
        }

        if(holdTurret) {
            r.turret.setYaw(0);
        }else {
            r.turret.face(a.pose,
                    futurePose,
                    r.drive.getFollower().getAngularVelocity());
        }
        double dis = PoseController.getGoalDis(futurePose,a);
        r.shooter.setDis(dis);
        r.hood.setDis(dis);

        if(helper.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0){
            r.intake.spinOut();
            r.gate.open_gate();
        }else if(helper.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            r.intake.spinIn();
            r.gate.close_gate();
        }else if(helper.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 ||
                base.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0){
            r.intake.spinIn();
            r.gate.open_gate();
            if(!compress) {
                compress = true;
                gating.reset();
            }
            r.indicator.set(IndicatorLed.Color.GREEN);
        }
        else{
            r.intake.spinOff();
            r.gate.close_gate();
            compress = false;
            r.indicator.set(IndicatorLed.Color.RED);
        }




        if(helper.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            r.shooter.shooterToggle();
            if(Shooter.activated){
                helper.gamepad.rumble(100);
            }
        }

        if(base.wasJustPressed(GamepadKeys.Button.START)){
            r.drive.cornerReset();
        }
        if(base.wasJustPressed(GamepadKeys.Button.TRIANGLE)){
            r.drive.goalReset();
        }
        if(base.getButton(GamepadKeys.Button.CROSS)) {
            r.drive.holdAngle();
        }else{
            r.drive.unholdAngle();
        }
        r.drive.face(a.pose,futurePose);

        packet.put("s vel",r.shooter.getVelocity());
        packet.put("s target vel",r.shooter.getTarget());
        packet.put("distance",dis);
        packet.put("Pose",r.drive.getPose());

        dash.sendTelemetryPacket(packet);

    }
}
