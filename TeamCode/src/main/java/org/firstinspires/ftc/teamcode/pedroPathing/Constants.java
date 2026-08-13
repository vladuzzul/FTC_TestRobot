package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants =
            new FollowerConstants();

    public static MecanumConstants driveConstants =
            new MecanumConstants();

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .hardwareMapName("pinpoint")
                    .distanceUnit(DistanceUnit.INCH)
                    .forwardPodY(0.0)
                    .strafePodX(0.0)
                    .encoderResolution(
                            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(
                            GoBildaPinpointDriver.EncoderDirection.FORWARD)
                    .strafeEncoderDirection(
                            GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
