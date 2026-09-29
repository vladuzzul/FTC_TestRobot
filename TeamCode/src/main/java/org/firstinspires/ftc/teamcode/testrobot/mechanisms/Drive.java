package org.firstinspires.ftc.teamcode.testrobot.mechanisms;

import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.AIM_INTEGRAL_LIMIT;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.AIM_KI;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.BLUE_BASKET_X;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.COMMON_BASKET_Y;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.RED_BASKET_X;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.AIM_KP;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.AIM_KD;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.AIM_DEADBAND_RAD;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;
import org.firstinspires.ftc.teamcode.testrobot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.testrobot.utils.FieldDrawing;
import org.firstinspires.ftc.teamcode.testrobot.utils.Constants;

import com.bylazar.telemetry.PanelsTelemetry;

/** Manual mecanum control and PedroPathing movement commands. */
public final class Drive implements Mechanism {
    private Follower follower;
    private Command defaultCommand = infinite(() -> {});
    private final FieldDrawing fieldDrawing = new FieldDrawing();
    private double appliedForward, appliedStrafe, appliedTurn;
    private Pose targetPose;

    private boolean fieldCentric = false;

    // For target aim
    private boolean aimingAtTarget = false;

    public boolean aimingAtBlue = true;
    private double targetX;
    private double targetY;

    private double integral = 0;
    private double lastError = 0;
    private long lastTime = System.nanoTime();
    private boolean hasLastAimError = false;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public void initialize(HardwareMap hardwareMap, Pose startingPose) {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        targetPose = null;
        fieldCentric = false;
        aimingAtTarget = false;
        aimingAtBlue = true;
        appliedForward = appliedStrafe = appliedTurn = 0;
        resetAimPid();
        defaultCommand = infinite(() -> {});
        fieldDrawing.initialize();
        periodic();
    }

    public boolean isInitialized() { return follower != null; }

    public Follower getFollower() { return follower; }

    public Pose getPose() { return follower.getPose(); }

    public void setPose(Pose pose) { follower.setPose(pose); }

    @Override
    public void periodic() {
        if (follower == null) return;
        follower.update();
        fieldDrawing.draw(follower.getPose());
    }

    @Override
    public Command getDefaultCommand() { return defaultCommand; }

    public void configureManual(Gamepad gamepad) {
        // Resumes after an instant setting command, without restarting Pedro's path follower.
        defaultCommand = infinite(() -> runManual(gamepad));
    }

    public Command manual() { return instant(this::startManual); }

    /** Keep ownership while Pedro holds the destination; Circle hands control back. */
    public Command driveToAndHold(Pose destination) {
        return Command.build().requiring(this)
                .setStart(() -> driveTo(destination))
                .setEnd(reason -> stop());
    }

    public Command followPath(PathChain path, Pose destination, double timeoutSeconds) {
        return new FollowPathCommand(this,
                () -> {
                    targetPose = destination;
                    follower.followPath(path, Constants.PATH_MAX_POWER, true);
                },
                () -> hasArrived(destination), this::stop, timeoutSeconds);
    }

    /** Build from the live pose when scheduled, including the emergency parking route. */
    public Command driveToCommand(Pose destination, double timeoutSeconds) {
        return new FollowPathCommand(this, () -> driveTo(destination),
                () -> hasArrived(destination), this::stop, timeoutSeconds);
    }

    private boolean hasArrived(Pose destination) {
        return !follower.isBusy() && follower.getPose().distanceFrom(destination) < 1.5
                && Math.abs(follower.getHeadingError()) < Math.toRadians(5);
    }

    public void stop() {
        if (follower != null) follower.breakFollowing();
    }

    /** Release per-OpMode references so the discovered Robot can be reused safely. */
    public void release() {
        stop();
        follower = null;
        defaultCommand = infinite(() -> {});
    }

    public double getAppliedForward() { return appliedForward; }
    public double getAppliedStrafe() { return appliedStrafe; }
    public double getAppliedTurn() { return appliedTurn; }

    public void startManual() {
        targetPose = null;
        appliedForward = appliedStrafe = appliedTurn = 0;
        follower.setMaxPowerScaling(1.0);
        follower.startTeleopDrive(true);
    }

    private void runManual(Gamepad gamepad) {
        if (!isManual()) {
            return;
        }

        appliedForward = Constants.applyDeadzone(-gamepad.left_stick_y);
        appliedStrafe = Constants.applyDeadzone(-gamepad.left_stick_x);
        appliedTurn = Constants.applyDeadzone(-gamepad.right_stick_x);
        follower.setTeleOpDrive(appliedForward, appliedStrafe,
                aimingAtTarget ? getAimTurn() : appliedTurn, !fieldCentric);
    }

    private void driveTo(Pose destination) {
        Pose currentPose = follower.getPose();
        targetPose = new Pose(
                destination.getX(), destination.getY(), destination.getHeading());

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(
                        currentPose.getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(path, Constants.PATH_MAX_POWER, true);
    }

    public void toggleFieldCentric(){
        fieldCentric = !fieldCentric;
    }
    public boolean isFieldCentric(){
        return fieldCentric;
    }

    public boolean isManual() {
        return follower.isTeleopDrive();
    }

    public void telemetry(Telemetry telemetry) {
        Pose pose = follower.getPose();
        String mode = isManual()
                ? "Manual"
                : (follower.isBusy() ? "Following path" : "Holding pose");

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
        resetAimPid();
        aimingAtTarget = true;
    }

    public void stopAiming() {
        aimingAtTarget = false;
        resetAimPid();
    }

    public boolean isAiming(){
        return aimingAtTarget;
    }

    public void toggleAimingTarget(){
        aimingAtBlue = !aimingAtBlue;
        targetX = aimingAtBlue ? BLUE_BASKET_X : RED_BASKET_X;
        resetAimPid();
    }

    private double getAimTurn() {
        Pose pose = follower.getPose();

        double desiredHeading = Math.atan2(
                targetY - pose.getY(),
                targetX - pose.getX()
        );

        double error = Math.atan2(
                Math.sin(desiredHeading - pose.getHeading()),
                Math.cos(desiredHeading - pose.getHeading())
        );

        if (Math.abs(error) < AIM_DEADBAND_RAD) {
            integral = 0.0;
            lastError = error;
            hasLastAimError = true;
            return 0.0;
        }

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.01;
        }

        integral += error * dt;
        integral = Math.max(
                -AIM_INTEGRAL_LIMIT,
                Math.min(AIM_INTEGRAL_LIMIT, integral)
        );

        double derivative = 0.0;
        if (hasLastAimError) {
            double errorDelta = Math.atan2(
                    Math.sin(error - lastError),
                    Math.cos(error - lastError)
            );
            derivative = errorDelta / dt;
        }

        lastError = error;
        hasLastAimError = true;

        double output =
                AIM_KP * error
                + AIM_KI * integral
                + AIM_KD * derivative;
        sendPanelsTelemetry(error, desiredHeading, output);
        return Math.max(-1.0, Math.min(1.0, output));
    }

    private void resetAimPid() {
        integral = 0.0;
        lastError = 0.0;
        lastTime = System.nanoTime();
        hasLastAimError = false;
    }

    private void sendPanelsTelemetry(double error, double desiredHeading, double output){
        panelsTelemetry.addData("Aim Error", error);
        panelsTelemetry.addData("Aim Desired", desiredHeading);
        panelsTelemetry.addData("Aim Output", output);
        panelsTelemetry.update();
    }
}
