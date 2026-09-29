package org.firstinspires.ftc.teamcode.testrobot.utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.testrobot.utils.RecordingFormat.*;

/** The original 21-column recording contract, readable without an Android device. */
public final class Recording {
    private static final int CSV_COLUMN_COUNT = COLUMN_COUNT;
    private Recording() {}

    public static List<RobotFrame> load(File file) throws IOException {
        List<RobotFrame> frames = new ArrayList<>();
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
                    || !"AngularVelocity".equals(headerColumns[17])
                    || !"FieldCentric".equals(headerColumns[18])
                    || !"AimingAtTarget".equals(headerColumns[19])
                    || !"AimingAtBlue".equals(headerColumns[20])) {
                throw new IOException("Unsupported CSV format; expected RecorderOp 21-column data");
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
                    // Every numeric column can eventually influence a drive command.
                    for (String value : values) {
                        if (!"true".equalsIgnoreCase(value) && !"false".equalsIgnoreCase(value)
                                && !Double.isFinite(Double.parseDouble(value))) {
                            throw new IOException("Non-finite value on CSV line " + lineNumber);
                        }
                    }
                    RobotFrame frame = new RobotFrame(values);
                    validateFrame(frame, lineNumber, frames);
                    frames.add(frame);
                } catch (NumberFormatException e) {
                    throw new IOException("Invalid number on CSV line " + lineNumber, e);
                }
            }
        }

        if (frames.isEmpty()) {
            throw new IOException("Recording contains no frames");
        }
        return frames;
    }

    private static void validateFrame(RobotFrame frame, int lineNumber, List<RobotFrame> frames) throws IOException {
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

    public static final class DriveCommand {
        public double forward;
        public double strafe;
        public double turn;

        public DriveCommand(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }

        public void scale(double amount) {
            forward = Math.max(-1, Math.min(1, forward * amount));
            strafe = Math.max(-1, Math.min(1, strafe * amount));
            turn = Math.max(-1, Math.min(1, turn * amount));
        }
    }

    public static final class RobotFrame {
        public final double timestamp;
        public final double leftRearPower;
        public final double rightRearPower;
        public final double leftFrontPower;
        public final double rightFrontPower;
        public final double x;
        public final double y;
        public final double heading;
        public final double voltage;
        public final double driveForward;
        public final double driveStrafe;
        public final double driveTurn;
        public final int driveMode;
        public final boolean aPressed;
        public final boolean bPressed;
        public final double velocityX;
        public final double velocityY;
        public final double angularVelocity;
        public final boolean fieldCentric;
        public final boolean aimingAtTarget;
        public final boolean aimingAtBlue;

        public RobotFrame(String[] values) {
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
            fieldCentric = parseBoolean(values[18]);
            aimingAtTarget = parseBoolean(values[19]);
            aimingAtBlue = parseBoolean(values[20]);
        }

        public DriveCommand feedforward() {
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
