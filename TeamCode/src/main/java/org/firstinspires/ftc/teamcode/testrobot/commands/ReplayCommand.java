package org.firstinspires.ftc.teamcode.testrobot.commands;

import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.AIM_DEADBAND_RAD;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.AIM_INTEGRAL_LIMIT;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.AIM_KD;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.AIM_KI;
import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.AIM_KP;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.testrobot.mechanisms.Drive;
import org.firstinspires.ftc.teamcode.testrobot.utils.Constants;
import org.firstinspires.ftc.teamcode.testrobot.utils.Recording;
import org.firstinspires.ftc.teamcode.testrobot.utils.Recording.RobotFrame;
import org.firstinspires.ftc.teamcode.testrobot.utils.Recording.DriveCommand;

import java.io.File;
import java.io.IOException;
import java.util.List;

/** Replays the 21-column CSV written by RecorderOp. */
public final class ReplayCommand extends CommandBuilder {

    private static final int MODE_MANUAL = 0;
    private static final int MODE_FOLLOWING_PATH = 1;
    private static final double TRANSLATION_KP = 0.022;
    private static final double TRANSLATION_KD = 0.013;
    private static final double ROTATION_KP = 0.30;
    private static final double ROTATION_KD = 0.08;
    private static final double VELOCITY_FILTER_ALPHA = 0.25;
    private static final double MAX_TRANSLATION_CORRECTION = 0.35;
    private static final double MAX_ROTATION_CORRECTION = 0.30;
    private static final double MIN_VOLTAGE_SCALE = 0.90;
    private static final double MAX_VOLTAGE_SCALE = 1.10;
    private static final double VOLTAGE_REFRESH_SEC = 0.10;
    private static final double MIN_DT = 0.008;
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = System.nanoTime();
    private boolean hasLastAimError = false;

    private boolean prevAiming = false;
    private boolean prevTargetBlue = false;

    private final Drive drive;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final List<RobotFrame> frames;
    private final ElapsedTime replayTimer = new ElapsedTime();

    private double currentVoltage = 13.0;
    private double lastVoltageReadTime = -10;

    private int frameIndex;
    private double previousLoopTime, previousHeading;
    private double filteredVelocityErrorX, filteredVelocityErrorY, filteredOmegaError;
    private double peakPositionError, meanPositionError, ticks;

    public ReplayCommand(File file, Drive drive, HardwareMap hardwareMap, Telemetry telemetry)
            throws IOException {
        this.drive = drive;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        frames = Recording.load(file);
        requiring(drive);
        setStart(this::beginReplay);
        setExecute(this::tick);
        setDone(() -> replayTimer.seconds() > duration());
        setEnd(reason -> drive.stop());
    }

    public Pose startingPose() {
        RobotFrame first = frames.get(0);
        return new Pose(first.x, first.y, first.heading);
    }

    public int frameCount() { return frames.size(); }

    private void beginReplay() {
        frameIndex = 0;
        previousLoopTime = 0;
        previousHeading = frames.get(0).heading;
        filteredVelocityErrorX = filteredVelocityErrorY = filteredOmegaError = 0;
        peakPositionError = meanPositionError = ticks = 0;
        lastVoltageReadTime = -10;
        prevAiming = false;
        prevTargetBlue = false;
        resetAimPid();
        drive.setPose(startingPose());
        drive.startManual();
        replayTimer.reset();
    }

    private void tick() {
        double replayDuration = duration();
        if (replayTimer.seconds() > replayDuration) return;
        double firstTimestamp = frames.get(0).timestamp;
        double now = replayTimer.seconds();
        double targetTimestamp = firstTimestamp + now;

        while (frameIndex < frames.size() - 1
                && frames.get(frameIndex + 1).timestamp <= targetTimestamp) {
            frameIndex++;
        }

        RobotFrame a = frames.get(frameIndex);
        RobotFrame b = frameIndex + 1 < frames.size()
                ? frames.get(frameIndex + 1) : a;
        double interpolation = interpolationAmount(a, b, targetTimestamp);

        double targetX = lerp(a.x, b.x, interpolation);
        double targetY = lerp(a.y, b.y, interpolation);
        double targetHeading = lerpAngle(a.heading, b.heading, interpolation);
        double targetVelocityX = lerp(a.velocityX, b.velocityX, interpolation);
        double targetVelocityY = lerp(a.velocityY, b.velocityY, interpolation);
        double targetOmega = lerp(a.angularVelocity, b.angularVelocity, interpolation);

        refreshVoltage(now);
        double recordedVoltage = lerp(a.voltage, b.voltage, interpolation);
        double voltageScale = currentVoltage > 1 && recordedVoltage > 1
                ? Range.clip(recordedVoltage / currentVoltage,
                MIN_VOLTAGE_SCALE, MAX_VOLTAGE_SCALE)
                : 1.0;

        DriveCommand feedforward = interpolateFeedforward(a, b, interpolation);
        // Field-centric input and target lock only affect manual driving.
        // Path/pose feedforward is already in the robot coordinate frame.
        boolean manualDrive = a.driveMode == MODE_MANUAL;
        boolean fieldCentric = manualDrive && a.fieldCentric;
        boolean aimingAtTarget = manualDrive && a.aimingAtTarget;
        feedforward.scale(voltageScale);

        Pose currentPose = drive.getFollower().getPose();
        Vector currentVelocity = drive.getFollower().getVelocity();
        double currentHeading = currentPose.getHeading();
        double dt = previousLoopTime > 0
                ? Math.max(now - previousLoopTime, MIN_DT) : MIN_DT;

        double errorFieldX = targetX - currentPose.getX();
        double errorFieldY = targetY - currentPose.getY();
        double headingError = normalizeAngle(targetHeading - currentHeading);
        double velocityErrorX = targetVelocityX - currentVelocity.getXComponent();
        double velocityErrorY = targetVelocityY - currentVelocity.getYComponent();
        double actualOmega = previousLoopTime > 0
                ? normalizeAngle(currentHeading - previousHeading) / dt
                : targetOmega;
        double omegaError = targetOmega - actualOmega;

        filteredVelocityErrorX += VELOCITY_FILTER_ALPHA
                * (velocityErrorX - filteredVelocityErrorX);
        filteredVelocityErrorY += VELOCITY_FILTER_ALPHA
                * (velocityErrorY - filteredVelocityErrorY);
        filteredOmegaError += VELOCITY_FILTER_ALPHA
                * (omegaError - filteredOmegaError);

        double correctionFieldX = errorFieldX * TRANSLATION_KP
                + filteredVelocityErrorX * TRANSLATION_KD;
        double correctionFieldY = errorFieldY * TRANSLATION_KP
                + filteredVelocityErrorY * TRANSLATION_KD;

        double cosHeading = Math.cos(currentHeading);
        double sinHeading = Math.sin(currentHeading);
        double correctionForward = cosHeading * correctionFieldX
                + sinHeading * correctionFieldY;
        double correctionStrafe = -sinHeading * correctionFieldX
                + cosHeading * correctionFieldY;

        double correctionMagnitude = Math.hypot(
                correctionForward, correctionStrafe);
        if (correctionMagnitude > MAX_TRANSLATION_CORRECTION) {
            double correctionScale = MAX_TRANSLATION_CORRECTION
                    / correctionMagnitude;
            correctionForward *= correctionScale;
            correctionStrafe *= correctionScale;
        }

        double correctionTurn = Range.clip(
                headingError * ROTATION_KP
                        + filteredOmegaError * ROTATION_KD,
                -MAX_ROTATION_CORRECTION, MAX_ROTATION_CORRECTION);

        double commandForward = feedforward.forward
                + (fieldCentric ? correctionFieldX : correctionForward);
        double commandStrafe = feedforward.strafe
                + (fieldCentric ? correctionFieldY : correctionStrafe);
        if (aimingAtTarget != prevAiming || a.aimingAtBlue != prevTargetBlue) {
            resetAimPid();
        }
        prevAiming = aimingAtTarget;
        prevTargetBlue = a.aimingAtBlue;
        double commandTurn = Range.clip(
                (aimingAtTarget ? aimTurn(currentPose, a.aimingAtBlue)
                        : feedforward.turn) + correctionTurn,
                -1, 1);
        double translationMagnitude = Math.hypot(
                commandForward, commandStrafe);
        if (translationMagnitude > 1) {
            commandForward /= translationMagnitude;
            commandStrafe /= translationMagnitude;
        }

        drive.getFollower().setTeleOpDrive(
                commandForward, commandStrafe, commandTurn, !fieldCentric);
        ticks++;

        double positionError = Math.hypot(errorFieldX, errorFieldY);
        meanPositionError += positionError;
        peakPositionError = Math.max(peakPositionError, positionError);
        telemetry.addData("Time", "%.2f / %.2f s", now, replayDuration);
        telemetry.addData("Frame", "%d / %d", frameIndex + 1, frames.size());
        telemetry.addData("Recorded mode", modeName(a.driveMode));
        telemetry.addData("Drive frame", fieldCentric ? "Field centric" : "Robot centric");
        telemetry.addData("Target lock", targetLockName(a));
        telemetry.addData("Recorded event", eventName(a));
        telemetry.addData("Position error", "%.2f in", positionError);
        telemetry.addData("Mean error", "%.2f in", meanPositionError / ticks);
        telemetry.addData("Peak error", "%.2f in", peakPositionError);
        telemetry.addData("Heading error", "%.1f deg",
                Math.toDegrees(Math.abs(headingError)));
        telemetry.addData("Feedforward", "%.2f  %.2f  %.2f",
                feedforward.forward, feedforward.strafe, feedforward.turn);
        telemetry.addData("Command", "%.2f  %.2f  %.2f",
                commandForward, commandStrafe, commandTurn);

        previousHeading = currentHeading;
        previousLoopTime = now;
    }

    private DriveCommand interpolateFeedforward(
            RobotFrame a, RobotFrame b, double amount) {
        DriveCommand commandA = a.feedforward();
        if (a.driveMode != b.driveMode
                || a.fieldCentric != b.fieldCentric
                || a.aimingAtTarget != b.aimingAtTarget
                || a.aimingAtBlue != b.aimingAtBlue) {
            return commandA;
        }

        DriveCommand commandB = b.feedforward();
        return new DriveCommand(
                lerp(commandA.forward, commandB.forward, amount),
                lerp(commandA.strafe, commandB.strafe, amount),
                lerp(commandA.turn, commandB.turn, amount));
    }

    private double interpolationAmount(
            RobotFrame a, RobotFrame b, double targetTimestamp) {
        double frameDuration = b.timestamp - a.timestamp;
        if (frameDuration <= 0) {
            return 0;
        }
        return Range.clip(
                (targetTimestamp - a.timestamp) / frameDuration, 0, 1);
    }

    private void refreshVoltage(double now) {
        if (now - lastVoltageReadTime >= VOLTAGE_REFRESH_SEC) {
            currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageReadTime = now;
        }
    }

    public double duration() {
        return frames.get(frames.size() - 1).timestamp
                - frames.get(0).timestamp;
    }

    private static String modeName(int driveMode) {
        if (driveMode == MODE_MANUAL) return "Manual";
        if (driveMode == MODE_FOLLOWING_PATH) return "Following path";
        return "Holding pose";
    }

    private static String eventName(RobotFrame frame) {
        if (frame.aPressed) return "A: drive to center";
        if (frame.bPressed) return "B: manual";
        return "-";
    }

    private static String targetLockName(RobotFrame frame) {
        if (!frame.aimingAtTarget) return "Off";
        return frame.aimingAtBlue ? "Blue" : "Red";
    }

    private double aimTurn(Pose pose, boolean aimingAtBlue) {
        double targetX = aimingAtBlue ? Constants.BLUE_BASKET_X : Constants.RED_BASKET_X;
        double desiredHeading = Math.atan2(
                Constants.COMMON_BASKET_Y - pose.getY(), targetX - pose.getX());
        double error = normalizeAngle(desiredHeading - pose.getHeading());

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

        return Range.clip(output, -1, 1);
    }

    private void resetAimPid() {
        integral = 0.0;
        lastError = 0.0;
        lastTime = System.nanoTime();
        hasLastAimError = false;
    }

    private static double lerp(double a, double b, double amount) {
        return a + (b - a) * amount;
    }

    private static double lerpAngle(double a, double b, double amount) {
        return normalizeAngle(a + normalizeAngle(b - a) * amount);
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

}
