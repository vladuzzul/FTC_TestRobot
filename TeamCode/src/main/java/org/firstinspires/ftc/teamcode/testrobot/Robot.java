package org.firstinspires.ftc.teamcode.testrobot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.testrobot.utils.components.DriveController;

/** Robot-level facade. Add future subsystems here, next to {@link #drive}. */
public final class Robot extends TestRobot {
    private static Robot instance;

    public final DriveController drive;

    private Robot() {
        drive = new DriveController(this);
    }

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    public void init(HardwareMap hardwareMap, Pose startingPose) {
        initFollower(hardwareMap, startingPose);
        drive.init();
    }

    public void updateAllSystems() {
        follower.update();
    }
}
