package org.firstinspires.ftc.teamcode.auto.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-35.41482226756982)
            .lateralZeroPowerAcceleration(-61.02340458740202)

            .headingPIDFCoefficients(new PIDFCoefficients(1.615, 0, 0.15, 0.015))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.4,0,0.095,0.025))

//            .translationalPIDFCoefficients(new PIDFCoefficients(0.095, 0, 0.0095, 0.0076))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.19, 0, 0.0209, 0))
//            .translationalPIDFSwitch(6)
//
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03325,0.0,0.0000095,0.6,0.19))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.04275, 0, 0.00000665, 0.6, 0.0))

            .centripetalScaling(0)

            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.09, 0.041041995519923375, 0.0019753949352179))

//            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
//            .useSecondaryDrivePIDF(true)
            .mass(11.7);

    public static PathConstraints pathConstraints = new PathConstraints(0.96, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4.291338583)
            .strafePodX(-6.88976378)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .maxPower(1)
            .useBrakeModeInTeleOp(true)
            .xVelocity(83.986)
            .yVelocity(58.119)
            .useVoltageCompensation(true);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
