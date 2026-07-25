package org.firstinspires.ftc.teamcode.teleop.comp.bex;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.robot.leds.IndicatorLed;
import org.firstinspires.ftc.teamcode.robot.utils.Alliance;
import org.firstinspires.ftc.teamcode.robot.utils.PoseController;
import org.firstinspires.ftc.teamcode.teleop.DennyOpMode;

@Disabled
@TeleOp(name = "BEX RED",group = "1111")
public class BexRed extends DennyOpMode {
    TelemetryPacket packet;
    FtcDashboard dash;
    boolean holdTurret = false;
    ElapsedTime gating;
    ElapsedTime artiing;
    ElapsedTime checkerTreshhold;
    boolean compress = false;
    boolean is3 = false;
    double maxPing = 0;
    static final double GOAL_ANGLE_STEP_DEG = 1.0;
    static final double SHOOTER_SPEED_STEP = 5.0;
    double goalAngleOffsetDeg = 0;
    @Override
    public void onInit() {
        super.onInit();
        packet = new TelemetryPacket();
        dash = FtcDashboard.getInstance();
        gating = new ElapsedTime();
        artiing = new ElapsedTime();
        checkerTreshhold = new ElapsedTime();
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

        r.drive.setRoboCentric();
//        r.prism.getSineWave().setDirection(Direction.Forward);
//        r.prism.getSineWave().setPeriod(50);
//        r.prism.getSineWave().setPrimaryColor(100,0,100);
//        r.prism.getSineWave().setBrightness(100);
//        r.prism.getSineWave().setSpeed(3);

        r.indicator.set(IndicatorLed.Color.RED);
    }

    @Override
    public void onUpdate() {

        if(base.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            goalAngleOffsetDeg += GOAL_ANGLE_STEP_DEG;
        }
        if(base.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            goalAngleOffsetDeg -= GOAL_ANGLE_STEP_DEG;
        }
        if(base.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            r.shooter.addVelOffset(SHOOTER_SPEED_STEP);
        }
        if(base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            r.shooter.addVelOffset(-SHOOTER_SPEED_STEP);
        }
        if(base.wasJustPressed(GamepadKeys.Button.SHARE)){
            goalAngleOffsetDeg = 0;
            holdTurret = false;
            r.shooter.resetVelOffset();
        }

        r.turret.unlock();

//        if(base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
//            r.drive.teleToggleCentric();
//        }     scibidi toilet SCIBIDITOILET
        r.drive.drive(base);

        Pose futurePose
//                = r.drive.getFollower().getPose();
                = PoseController.getFuturePose(r.drive.getFollower());


        if(base.wasJustPressed(GamepadKeys.Button.B)){
            holdTurret = !holdTurret;
        }

        Pose goalPose ;
        if(PoseController.isInFarZone(r.drive.getPose())){
            r.hood.setCompMode(true);
            goalPose = a.farPose;
        }else{
            r.hood.setCompMode(false);
            goalPose = a.pose;
        }

        if(holdTurret) {
            r.turret.setYaw(0);
        }else {
            r.turret.face(goalPose,
                    futurePose,
                    r.drive.getFollower().getAngularVelocity(),
                    Math.toRadians(goalAngleOffsetDeg));
        }
        double dis = PoseController.getGoalDis(futurePose,a);
        r.shooter.setDis(dis);
        r.hood.setDis(dis);



        r.drive.setSlower(1);
        if(base.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0){
            r.intake.spinOut();
            r.gate.open_gate();
        }else if(base.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            r.intake.spinIn();
            r.gate.close_gate();
            r.intake.transferOff();
//            r.drive.setSlower(0.45);
        }else if(base.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0){
            if(PoseController.isInFarZone(r.drive.getPose())){
                r.intake.spinShoot();
                r.turret.lock();
            }
            else {
                r.intake.spinShoot();
            }
            r.gate.open_gate();
            if(!compress) {
                compress = true;
                gating.reset();
            }
            r.indicator.set(IndicatorLed.Color.GREEN);
            is3 = false;
        }
        else{
            r.intake.spinOff();
            r.gate.close_gate();
            compress = false;
            r.indicator.set(IndicatorLed.Color.RED);
        }
        if(base.wasJustPressed(GamepadKeys.Button.START)){
            r.drive.cornerReset();
        }
        if(base.wasJustPressed(GamepadKeys.Button.TRIANGLE)){
            r.drive.goalReset();
        }
        if(base.getButton(GamepadKeys.Button.CROSS)) {
            r.drive.holdAngle();
            r.drive.face(a.pose,futurePose);
        }
        else if(base.getButton(GamepadKeys.Button.SQUARE)) {
            r.drive.holdAngle();
            r.drive.setAngle(a.equals(Alliance.RED)?Math.toRadians(30):Math.toRadians(150));
        }else{
            r.drive.unholdAngle();
        }

        if(is3)
            base.gamepad.rumble(67);

        if(maxPing < deltaTime){
            maxPing = deltaTime;
        }

        if(PoseController.isInZone(r.drive.getPose())){
            r.intake.transfOn();
        }

        packet.put("distance",dis);
        packet.put("Pose",r.drive.getPose());
        packet.put("Loop Time",deltaTime);
        packet.put("Max Loop Time",maxPing);

        packet.put("Angel velocity",r.drive.getFollower().getAngularVelocity());

        packet.put("s vel",r.shooter.getVelocity());
        packet.put("s target vel",r.shooter.getTarget());

        packet.put("heading",r.drive.getFollower().getHeading());
        packet.put("s heading ++", r.drive.heading);
        dash.sendTelemetryPacket(packet);

    }
}
