package org.firstinspires.ftc.teamcode.testrobot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.testrobot.utils.components.DriveController;
import org.firstinspires.ftc.teamcode.testrobot.utils.components.Intake;

public final class Robot extends TestRobot {
    private static Robot instance;

    public final DriveController drive;
//    public final Intake intake;

    private Robot() {
        drive = new DriveController(this);
//        intake = new Intake();
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
//        intake.init(hardwareMap);
    }

    public void updateAllSystems() {
        follower.update();
    }
}
