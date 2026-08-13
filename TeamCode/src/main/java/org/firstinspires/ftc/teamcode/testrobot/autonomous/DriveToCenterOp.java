package org.firstinspires.ftc.teamcode.testrobot.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.testrobot.baseOps.BaseAuto;
import org.firstinspires.ftc.teamcode.testrobot.utils.Constants;
import org.firstinspires.ftc.teamcode.testrobot.utils.PoseStorage;

/** Autonomous test that drives from the left start pose to field center. */
@Autonomous(name = "TestRobot Drive To Center", group = Constants.MAIN_GROUP)
public class DriveToCenterOp extends BaseAuto {
    @Override
    protected Pose startingPose() {
        return PoseStorage.loadPose();
    }

    @Override
    protected void onAutoStart() {
        robot.drive.driveTo(AutonomousConstants.centerPose());
    }
}
