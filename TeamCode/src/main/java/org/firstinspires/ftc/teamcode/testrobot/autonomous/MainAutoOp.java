package org.firstinspires.ftc.teamcode.testrobot.autonomous;

import static org.firstinspires.ftc.teamcode.testrobot.autonomous.AutonomousConstants.TIMEOUT_FAILSAFE;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.PATH_MAX_POWER;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.testrobot.baseOps.BaseAuto;
import org.firstinspires.ftc.teamcode.testrobot.utils.Constants;
import org.firstinspires.ftc.teamcode.testrobot.utils.PoseStorage;

@Autonomous(name = "Path Test", group = Constants.MAIN_GROUP)
public class MainAutoOp extends BaseAuto {

    private int pathState = -1;
    private final ElapsedTime stateTimer = new ElapsedTime();



    private final Pose startingPose = new Pose(79.5, 9, Math.toRadians(90));

    private final Pose forwardPose = new Pose(79.5, 130, Math.toRadians(90));
    private final Pose launchZone = new Pose(90, 115, Math.toRadians(40));
    private final Pose intake10 = new Pose(106, 34.5, Math.toRadians(0));
    private final Pose intake11 = new Pose(130, 34.5, Math.toRadians(0));
    private final Pose intake20 = new Pose(106, 58.5, Math.toRadians(0));
    private final Pose intake21 = new Pose(130, 58.5, Math.toRadians(0));
    private final Pose intake30 = new Pose(106, 82, Math.toRadians(0));
    private final Pose intake31 = new Pose(125, 82, Math.toRadians(0));

    private final Pose parkingPose = new Pose(103.5, 32.5, Math.toRadians(0));

    private PathChain goToIntake1, goIntake1, goIntake10, goToLaunch1;
    private PathChain goToIntake2, goIntake2, goIntake20, goToLaunch2;
    private PathChain goToIntake3, goIntake3, goIntake30, goToLaunch3;
    private PathChain gotToStart;
//    private PathChain goFront, goBack;
    @Override
    protected Pose startingPose() {
        return startingPose;
    }

    @Override
    public void onAutoInit() {
        goToIntake1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(startingPose, intake10))
                .setLinearHeadingInterpolation(startingPose.getHeading(), intake10.getHeading())
                .build();
        goIntake1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake10, intake11))
                .setLinearHeadingInterpolation(intake10.getHeading(), intake11.getHeading())
                .build();
        goIntake10 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake11, intake10))
                .setLinearHeadingInterpolation(intake11.getHeading(), intake10.getHeading())
                .build();
        goToLaunch1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake10, launchZone))
                .setLinearHeadingInterpolation(intake10.getHeading(), launchZone.getHeading())
                .build();

        goToIntake2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(launchZone, intake20))
                .setLinearHeadingInterpolation(launchZone.getHeading(), intake20.getHeading())
                .build();
        goIntake2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake20, intake21))
                .setLinearHeadingInterpolation(intake20.getHeading(), intake21.getHeading())
                .build();
        goIntake20 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake21, intake20))
                .setLinearHeadingInterpolation(intake21.getHeading(), intake20.getHeading())
                .build();
        goToLaunch2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake20, launchZone))
                .setLinearHeadingInterpolation(intake20.getHeading(), launchZone.getHeading())
                .build();

        goToIntake3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(launchZone, intake30))
                .setLinearHeadingInterpolation(launchZone.getHeading(), intake30.getHeading())
                .build();
        goIntake3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake30, intake31))
                .setLinearHeadingInterpolation(intake30.getHeading(), intake31.getHeading())
                .build();
        goIntake30 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake31, intake30))
                .setLinearHeadingInterpolation(intake31.getHeading(), intake30.getHeading())
                .build();
        goToLaunch3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intake30, launchZone))
                .setLinearHeadingInterpolation(intake30.getHeading(), launchZone.getHeading())
                .build();

        gotToStart = robot.follower.pathBuilder()
                .addPath(new BezierLine(launchZone, parkingPose))
                .setLinearHeadingInterpolation(launchZone.getHeading(), startingPose.getHeading())
                .build();
//        goFront = robot.follower.pathBuilder()
//                .addPath(new BezierLine(startingPose, forwardPose))
//                .setLinearHeadingInterpolation(startingPose.getHeading(), forwardPose.getHeading())
//                .build();
//        goBack = robot.follower.pathBuilder()
//                .addPath(new BezierLine(forwardPose, startingPose))
//                .setLinearHeadingInterpolation(forwardPose.getHeading(), startingPose.getHeading())
//                .build();
    }

    @Override
    public void onAutoStart() {
        setPathState(0);
    }

    @Override
    protected void onAutoLoop() {
        switch (pathState) {
            case 0:
                robot.follower.followPath(goToIntake1, PATH_MAX_POWER, true);
                setPathState(1);
                break;
            case 1:
                if (canTransition(intake10, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(goIntake1, PATH_MAX_POWER, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (canTransition(intake11, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(goIntake10, PATH_MAX_POWER, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (canTransition(intake10, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(goToLaunch1, PATH_MAX_POWER, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (canTransition(launchZone, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(goToIntake2, PATH_MAX_POWER, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (canTransition(intake20, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(goIntake2, PATH_MAX_POWER, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (canTransition(intake21, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(goIntake20, PATH_MAX_POWER, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (canTransition(intake20, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(goToLaunch2, PATH_MAX_POWER, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (canTransition(launchZone, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(goToIntake3, PATH_MAX_POWER, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (canTransition(intake30, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(goIntake3, PATH_MAX_POWER, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (canTransition(intake31, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(goIntake30, PATH_MAX_POWER, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (canTransition(intake30, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(goToLaunch3, PATH_MAX_POWER, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (canTransition(launchZone, TIMEOUT_FAILSAFE)) {
                    robot.follower.followPath(gotToStart, PATH_MAX_POWER, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (canTransition(parkingPose, TIMEOUT_FAILSAFE)) {
                    setPathState(-1);
                }
                break;
        }

//        switch(pathState){
//            case 0:
//                robot.follower.followPath(goFront, PATH_MAX_POWER, true);
//                setPathState(1);
//                break;
//            case 1:
//                if (canTransition(forwardPose, 20.0)) {
//                    robot.follower.followPath(goBack, PATH_MAX_POWER, true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if (canTransition(startingPose, 20.0)) {
//                    setPathState(-1);
//                }
//                break;
//        }
        telemetry.addData("Path State", pathState);
        telemetry.addData("Is Busy", robot.follower.isBusy());
    }

    @Override
    protected void onAutoStop(){
        PoseStorage.savePose(parkingPose);
    }

    private void setPathState(int state) {
        pathState = state;
        stateTimer.reset();
    }
    private boolean canTransition(Pose target, double timeout) {
        double currentDist = robot.follower.getPose().distanceFrom(target);
        double headingError = Math.abs(robot.follower.getHeadingError());

        boolean arrived = !robot.follower.isBusy() && currentDist < 1.5 && headingError < Math.toRadians(5);

        boolean timedOut = stateTimer.seconds() > timeout;

        return arrived || timedOut;
    }
}
