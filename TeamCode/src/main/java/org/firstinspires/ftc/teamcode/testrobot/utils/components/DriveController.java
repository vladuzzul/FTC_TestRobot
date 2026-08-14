package org.firstinspires.ftc.teamcode.testrobot.utils.components;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.testrobot.Robot;
import org.firstinspires.ftc.teamcode.testrobot.utils.Constants;
import org.firstinspires.ftc.teamcode.testrobot.utils.Controls;

/** Manual mecanum control and PedroPathing movement commands. */
public final class DriveController {
    private final Robot robot;
    private Pose targetPose;

    private boolean fieldCentric = false;

    public DriveController(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        targetPose = null;
    }

    public void startManual() {
        targetPose = null;
        robot.follower.setMaxPowerScaling(1.0);
        robot.follower.startTeleopDrive(true);
    }

    public void runManual(Controls controls) {
        if (!isManual()) {
            return;
        }

        robot.follower.setTeleOpDrive(
                Constants.applyDeadzone(-controls.leftStickY),
                Constants.applyDeadzone(-controls.leftStickX),
                Constants.applyDeadzone(-controls.rightStickX),
                !fieldCentric);
    }

    public void driveTo(Pose destination) {
        Pose currentPose = robot.follower.getPose();
        targetPose = new Pose(
                destination.getX(), destination.getY(), destination.getHeading());

        PathChain path = robot.follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(
                        currentPose.getHeading(), targetPose.getHeading())
                .build();

        robot.follower.followPath(path, Constants.PATH_MAX_POWER, true);
    }

    public void toggleFieldCentric(){
        fieldCentric = !fieldCentric;
    }
    public boolean isFieldCentric(){
        return fieldCentric;
    }

    public boolean isManual() {
        return robot.follower.isTeleopDrive();
    }

    public void telemetry(Telemetry telemetry) {
        Pose pose = robot.follower.getPose();
        String mode = isManual()
                ? "Manual"
                : (robot.follower.isBusy() ? "Following path" : "Holding pose");

        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Mode", mode);
        telemetry.addData("X (in)", "%.2f", pose.getX());
        telemetry.addData("Y (in)", "%.2f", pose.getY());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));

        if (targetPose != null) {
            telemetry.addData("Target", "(%.1f, %.1f, %.0f deg)",
                    targetPose.getX(), targetPose.getY(),
                    Math.toDegrees(targetPose.getHeading()));
        }
    }
}
