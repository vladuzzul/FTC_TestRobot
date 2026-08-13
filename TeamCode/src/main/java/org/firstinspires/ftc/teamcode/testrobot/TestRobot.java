package org.firstinspires.ftc.teamcode.testrobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Container for the hardware services shared by all TestRobot OpModes.
 * PedroPathing owns both the mecanum drivetrain and the Pinpoint localizer.
 */
public class TestRobot {
    public Follower follower;

    public void initFollower(HardwareMap hardwareMap, Pose startingPose) {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants
                .createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
    }
}
