package org.firstinspires.ftc.teamcode.testrobot.utils.components;

import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.BLUE_BASKET_X;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.COMMON_BASKET_Y;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.RED_BASKET_X;

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

    // For target aim
    private boolean aimingAtTarget = false;

    public boolean aimingAtBlue = true;
    private double targetX;
    private double targetY;

    private static final double AIM_KP = 1.0;
    private static final double AIM_DEADBAND_RAD = Math.toRadians(2.0);

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

        if (!aimingAtTarget){
            robot.follower.setTeleOpDrive(
                    Constants.applyDeadzone(-controls.leftStickY),
                    Constants.applyDeadzone(-controls.leftStickX),
                    Constants.applyDeadzone(-controls.rightStickX),
                    !fieldCentric);
        }
        else {
            robot.follower.setTeleOpDrive(
                    Constants.applyDeadzone(-controls.leftStickY),
                    Constants.applyDeadzone(-controls.leftStickX),
                    getAimTurn(),
                    !fieldCentric);
        }
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

        if (isFieldCentric()){
            telemetry.addLine("Field centric");
        }
        else{
            telemetry.addLine("Robot centric");
        }

        if (targetPose != null) {
            telemetry.addData("Target", "(%.1f, %.1f, %.0f deg)",
                    targetPose.getX(), targetPose.getY(),
                    Math.toDegrees(targetPose.getHeading()));
        }

        if (aimingAtTarget){
            telemetry.addData("Aiming at", aimingAtBlue ? "Blue" : "Red");
            telemetry.addLine("Press R3 to switch target");
        }
    }

    public void startAiming() {
        if (aimingAtBlue){
            targetX = BLUE_BASKET_X;
        }
        else{
            targetX = RED_BASKET_X;
        }

        targetY = COMMON_BASKET_Y;
        aimingAtTarget = true;
    }

    public void stopAiming() {
        aimingAtTarget = false;
    }

    public boolean isAiming(){
        return aimingAtTarget;
    }

    public void toggleAimingTarget(){
        aimingAtBlue = !aimingAtBlue;
        targetX = aimingAtBlue ? BLUE_BASKET_X : RED_BASKET_X;
    }

    private double getAimTurn() {
        Pose pose = robot.follower.getPose();

        double desiredHeading = Math.atan2(
                targetY - pose.getY(),
                targetX - pose.getX());

        double error = Math.atan2(
                Math.sin(desiredHeading - pose.getHeading()),
                Math.cos(desiredHeading - pose.getHeading()));

        if (Math.abs(error) < AIM_DEADBAND_RAD) {
            return 0.0;
        }

        return Math.max(-1.0, Math.min(1.0, AIM_KP * error));
    }
}
