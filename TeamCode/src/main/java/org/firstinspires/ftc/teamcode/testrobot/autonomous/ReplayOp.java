package org.firstinspires.ftc.teamcode.testrobot.autonomous;

import android.os.Environment;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.testrobot.Robot;
import org.firstinspires.ftc.teamcode.testrobot.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.testrobot.utils.RecordingFormat;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/** Replays the 18-column CSV written by RecorderOp. */
@Autonomous(name = "Replay", group = "Replay")
public class ReplayOp extends LinearOpMode {
    private static final String CSV_FILE_NAME = RecordingFormat.FILE_NAME;
    private static final int CSV_COLUMN_COUNT = RecordingFormat.COLUMN_COUNT;

    private static final int MODE_MANUAL = 0;
    private static final int MODE_FOLLOWING_PATH = 1;
    private static final int MODE_HOLDING_POSE = 2;

    // Conservative correction values. Tune these on the real robot if needed.
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

    private final Robot robot = Robot.getInstance();
    private final List<RobotFrame> frames = new ArrayList<>();
    private final ElapsedTime replayTimer = new ElapsedTime();

    private File recordingFile;
    private boolean robotInitialized;
    private double currentVoltage = 13.0;
    private double lastVoltageReadTime = -10;

    @Override
    public void runOpMode() {
        try {
            recordingFile = new File(
                    Environment.getExternalStorageDirectory(), CSV_FILE_NAME);
            loadRecording(recordingFile);

            RobotFrame first = frames.get(0);
            robot.init(hardwareMap, new Pose(first.x, first.y, first.heading));
            robotInitialized = true;

            telemetry.addData("Replay file", recordingFile.getAbsolutePath());
            telemetry.addData("Frames", frames.size());
            telemetry.addData("Duration", "%.2f s", duration());
            telemetry.addLine("Ready to replay");
            telemetry.update();

            waitForStart();
            if (isStopRequested()) {
                return;
            }

            // Force the same coordinate origin used at the beginning of recording.
            robot.setPose(new Pose(first.x, first.y, first.heading));
            robot.updateAllSystems();
            robot.drive.startManual();

            executeReplay();
        } catch (Exception e) {
            telemetry.addData("Replay error", e.getMessage() == null
                    ? e.getClass().getSimpleName() : e.getMessage());
            telemetry.update();
        } finally {
            stopRobotSafely();
        }
    }

    private void executeReplay() {
        RobotFrame first = frames.get(0);
        double firstTimestamp = first.timestamp;
        double replayDuration = duration();
        int frameIndex = 0;

        double previousLoopTime = 0;
        double previousHeading = first.heading;
        double filteredVelocityErrorX = 0;
        double filteredVelocityErrorY = 0;
        double filteredOmegaError = 0;
        double peakPositionError = 0;
        double meanPositionError = 0;
        double ticks = 0;

        replayTimer.reset();

        while (opModeIsActive() && replayTimer.seconds() <= replayDuration) {
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
            feedforward.scale(voltageScale);

            Pose currentPose = robot.follower.getPose();
            Vector currentVelocity = robot.follower.getVelocity();
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

            double commandForward = feedforward.forward + correctionForward;
            double commandStrafe = feedforward.strafe + correctionStrafe;
            double commandTurn = Range.clip(
                    feedforward.turn + correctionTurn, -1, 1);
            double translationMagnitude = Math.hypot(
                    commandForward, commandStrafe);
            if (translationMagnitude > 1) {
                commandForward /= translationMagnitude;
                commandStrafe /= translationMagnitude;
            }

            robot.follower.setTeleOpDrive(
                    commandForward, commandStrafe, commandTurn, true);
            robot.updateAllSystems();
            ticks++;

            double positionError = Math.hypot(errorFieldX, errorFieldY);
            meanPositionError += positionError;
            peakPositionError = Math.max(peakPositionError, positionError);
            telemetry.addData("Time", "%.2f / %.2f s", now, replayDuration);
            telemetry.addData("Frame", "%d / %d", frameIndex + 1, frames.size());
            telemetry.addData("Recorded mode", modeName(a.driveMode));
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
            telemetry.update();

            previousHeading = currentHeading;
            previousLoopTime = now;
            idle();
        }
    }

    private void loadRecording(File file) throws IOException {
        if (!file.isFile()) {
            throw new IOException("Recording not found: " + file.getAbsolutePath());
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String header = reader.readLine();
            if (header == null) {
                throw new IOException("Recording is empty");
            }

            String[] headerColumns = header.split(",", -1);
            if (headerColumns.length != CSV_COLUMN_COUNT
                    || !"DriveMode".equals(headerColumns[12])
                    || !"AngularVelocity".equals(headerColumns[17])) {
                throw new IOException("Unsupported CSV format; expected RecorderOp 18-column data");
            }

            String line;
            int lineNumber = 1;
            while ((line = reader.readLine()) != null) {
                lineNumber++;
                if (line.trim().isEmpty()) {
                    continue;
                }

                String[] values = line.split(",", -1);
                if (values.length != CSV_COLUMN_COUNT) {
                    throw new IOException("Invalid column count on CSV line " + lineNumber);
                }

                try {
                    RobotFrame frame = new RobotFrame(values);
                    validateFrame(frame, lineNumber);
                    frames.add(frame);
                } catch (NumberFormatException e) {
                    throw new IOException("Invalid number on CSV line " + lineNumber, e);
                }
            }
        }

        if (frames.isEmpty()) {
            throw new IOException("Recording contains no frames");
        }
    }

    private void validateFrame(RobotFrame frame, int lineNumber) throws IOException {
        if (!Double.isFinite(frame.timestamp)
                || !Double.isFinite(frame.x)
                || !Double.isFinite(frame.y)
                || !Double.isFinite(frame.heading)) {
            throw new IOException("Non-finite value on CSV line " + lineNumber);
        }
        if (!frames.isEmpty()
                && frame.timestamp < frames.get(frames.size() - 1).timestamp) {
            throw new IOException("Timestamps go backwards on CSV line " + lineNumber);
        }
        if (frame.driveMode < MODE_MANUAL || frame.driveMode > MODE_HOLDING_POSE) {
            throw new IOException("Invalid DriveMode on CSV line " + lineNumber);
        }
    }

    private DriveCommand interpolateFeedforward(
            RobotFrame a, RobotFrame b, double amount) {
        DriveCommand commandA = a.feedforward();
        if (a.driveMode != b.driveMode) {
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

    private void stopRobotSafely() {
        if (!robotInitialized || robot.follower == null) {
            return;
        }

        robot.follower.setTeleOpDrive(0, 0, 0, true);
        robot.updateAllSystems();
        PoseStorage.savePose(robot.follower.getPose());
        robot.stop();
    }

    private double duration() {
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

    private static final class DriveCommand {
        double forward;
        double strafe;
        double turn;

        DriveCommand(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }

        void scale(double amount) {
            forward = Range.clip(forward * amount, -1, 1);
            strafe = Range.clip(strafe * amount, -1, 1);
            turn = Range.clip(turn * amount, -1, 1);
        }
    }

    private static final class RobotFrame {
        final double timestamp;
        final double leftRearPower;
        final double rightRearPower;
        final double leftFrontPower;
        final double rightFrontPower;
        final double x;
        final double y;
        final double heading;
        final double voltage;
        final double driveForward;
        final double driveStrafe;
        final double driveTurn;
        final int driveMode;
        final boolean aPressed;
        final boolean bPressed;
        final double velocityX;
        final double velocityY;
        final double angularVelocity;

        RobotFrame(String[] values) {
            timestamp = Double.parseDouble(values[0]);
            leftRearPower = Double.parseDouble(values[1]);
            rightRearPower = Double.parseDouble(values[2]);
            leftFrontPower = Double.parseDouble(values[3]);
            rightFrontPower = Double.parseDouble(values[4]);
            x = Double.parseDouble(values[5]);
            y = Double.parseDouble(values[6]);
            heading = Double.parseDouble(values[7]);
            voltage = Double.parseDouble(values[8]);
            driveForward = Double.parseDouble(values[9]);
            driveStrafe = Double.parseDouble(values[10]);
            driveTurn = Double.parseDouble(values[11]);
            driveMode = Integer.parseInt(values[12]);
            aPressed = parseBoolean(values[13]);
            bPressed = parseBoolean(values[14]);
            velocityX = Double.parseDouble(values[15]);
            velocityY = Double.parseDouble(values[16]);
            angularVelocity = Double.parseDouble(values[17]);
        }

        DriveCommand feedforward() {
            if (driveMode == MODE_MANUAL) {
                return new DriveCommand(driveForward, driveStrafe, driveTurn);
            }

            // Path-following and holding rows have zero joystick commands.
            // Recover their drive feedforward from the four recorded powers.
            return new DriveCommand(
                    (leftFrontPower + rightFrontPower
                            + leftRearPower + rightRearPower) / 4.0,
                    (leftFrontPower - rightFrontPower
                            - leftRearPower + rightRearPower) / 4.0,
                    (leftFrontPower - rightFrontPower
                            + leftRearPower - rightRearPower) / 4.0);
        }

        private static boolean parseBoolean(String value) {
            return "1".equals(value) || "true".equalsIgnoreCase(value);
        }
    }
}
