package org.firstinspires.ftc.teamcode.testrobot.opmodes.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.teamcode.testrobot.Robot;
import org.firstinspires.ftc.teamcode.testrobot.config.AutonomousConstants;
import org.firstinspires.ftc.teamcode.testrobot.opmodes.base.BaseAuto;
import org.firstinspires.ftc.teamcode.testrobot.utils.Constants;
import org.firstinspires.ftc.teamcode.testrobot.utils.PoseStorage;

@NextAutonomous(name = "TestRobot Drive To Center", group = Constants.MAIN_GROUP,
        preselectTeleop = "Main Driving")
public final class DriveToCenterOp extends BaseAuto {
    public DriveToCenterOp(Robot robot) { super(robot); }
    @Override protected Pose startingPose() { return PoseStorage.loadPose(); }
    @Override protected Command autonomousCommand() {
        return robot.drive.driveToAndHold(AutonomousConstants.centerPose());
    }
}
