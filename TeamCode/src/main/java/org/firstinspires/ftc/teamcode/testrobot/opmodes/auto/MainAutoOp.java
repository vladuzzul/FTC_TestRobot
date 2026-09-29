package org.firstinspires.ftc.teamcode.testrobot.opmodes.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;
import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.teamcode.testrobot.Robot;
import org.firstinspires.ftc.teamcode.testrobot.config.AutonomousConstants;
import org.firstinspires.ftc.teamcode.testrobot.opmodes.base.BaseAuto;
import org.firstinspires.ftc.teamcode.testrobot.utils.Constants;

@NextAutonomous(name = "Path Test", group = Constants.MAIN_GROUP, preselectTeleop = "Main Driving")
public final class MainAutoOp extends BaseAuto {
    private final Pose start = new Pose(79.5, 9, Math.toRadians(90));
    private final Pose launch = new Pose(90, 115, Math.toRadians(40));
    private final Pose intake10 = new Pose(106, 34.5, 0);
    private final Pose intake11 = new Pose(130, 34.5, 0);
    private final Pose intake20 = new Pose(106, 58.5, 0);
    private final Pose intake21 = new Pose(130, 58.5, 0);
    private final Pose intake30 = new Pose(106, 82, 0);
    private final Pose intake31 = new Pose(125, 82, 0);
    private final Pose parking = new Pose(103.5, 32.5, 0);
    private final ElapsedTime totalTimer = new ElapsedTime();

    public MainAutoOp(Robot robot) { super(robot); }
    @Override protected Pose startingPose() { return start; }

    @Override protected Command autonomousCommand() {
        Command cycles = Groups.sequential(
                leg(start, intake10), leg(intake10, intake11), leg(intake11, intake10), leg(intake10, launch),
                leg(launch, intake20), leg(intake20, intake21), leg(intake21, intake20), leg(intake20, launch),
                leg(launch, intake30), leg(intake30, intake31), leg(intake31, intake30), leg(intake30, launch));
        // A deadline interrupts the current path; the parking path is built from the live pose.
        return Groups.sequential(
                Commands.instant(totalTimer::reset),
                cycles.until(() -> totalTimer.seconds() > AutonomousConstants.PARKING_FAILSAFE),
                robot.drive.driveToCommand(parking, AutonomousConstants.STEP_FAILSAFE));
    }

    private Command leg(Pose from, Pose to) {
        PathChain path = robot.drive.getFollower().pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
        return robot.drive.followPath(path, to, AutonomousConstants.STEP_FAILSAFE);
    }
}
