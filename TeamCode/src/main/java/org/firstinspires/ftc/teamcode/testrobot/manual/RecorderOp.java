package org.firstinspires.ftc.teamcode.testrobot.manual;

import android.os.Environment;

import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.testrobot.baseOps.MainTeleOp;
import org.firstinspires.ftc.teamcode.testrobot.utils.Constants;
import org.firstinspires.ftc.teamcode.testrobot.utils.RecordingFormat;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name = "Recorder", group = "Replay")
public class RecorderOp extends MainTeleOp {
    private BufferedWriter dataRecorder;
    private static final double RECORD_INTERVAL_SEC = 0.02;
    private final ElapsedTime recordTimer = new ElapsedTime();
    private double lastRecordTime;
    private String recordingFilePath;
    private String recordingError;

    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
    private GoBildaPinpointDriver pinpoint;

    @Override
    protected void onRobotInit() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        File recordingFile = new File(
                Environment.getExternalStorageDirectory(),
                RecordingFormat.FILE_NAME);
        recordingFilePath = recordingFile.getAbsolutePath();

        try {
            dataRecorder = new BufferedWriter(
                    new FileWriter(recordingFile), 32 * 1024);
            dataRecorder.write(RecordingFormat.HEADER + "\n");
        } catch (IOException e) {
            handleRecordingError(e);
        }
    }

    @Override
    protected void onRobotStart() {
        recordTimer.reset();
        lastRecordTime = 0;
        recordFrame(0, true);
    }

    @Override
    protected void onRobotUpdated() {
        double sampleTime = recordTimer.seconds();
        boolean importantEvent = controls.cross.justPressed() || controls.circle.justPressed();
        recordFrame(sampleTime, importantEvent);
    }

    @Override
    protected void onRobotLoop() {
        telemetry.addData("Recording", dataRecorder == null ? "FAILED" : "ACTIVE");
        telemetry.addData("Recording file", recordingFilePath);
        if (recordingError != null) {
            telemetry.addData("Recording error", recordingError);
        }
    }

    @Override
    protected void onRobotStopping() {
        recordFrame(recordTimer.seconds(), true);
        closeRecorder();
    }

    private void recordFrame(double sampleTime, boolean force) {
        if (dataRecorder == null) {
            return;
        }
        if (!force && sampleTime - lastRecordTime < RECORD_INTERVAL_SEC) {
            return;
        }

        boolean manualDrive = robot.drive.isManual();
        double driveFwd = manualDrive
                ? Constants.applyDeadzone(-controls.leftStickY) : 0;
        double driveStr = manualDrive
                ? Constants.applyDeadzone(-controls.leftStickX) : 0;
        double driveTurn = manualDrive
                ? Constants.applyDeadzone(-controls.rightStickX) : 0;
        int driveMode = manualDrive
                ? RecordingFormat.MODE_MANUAL
                : (robot.follower.isBusy()
                ? RecordingFormat.MODE_FOLLOWING_PATH
                : RecordingFormat.MODE_HOLDING_POSE);

        try {
            recordData(sampleTime, driveFwd, driveStr, driveTurn, driveMode);
            lastRecordTime = sampleTime;
        } catch (IOException e) {
            handleRecordingError(e);
            closeRecorder();
        }
    }

    private void recordData(double t, double driveFwd, double driveStr,
                            double driveTurn, int driveMode) throws IOException {

        double x = robot.follower.getPose().getX();
        double y = robot.follower.getPose().getY();
        double heading = robot.follower.getPose().getHeading();

        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        Vector velocity      = robot.follower.getVelocity();
        double velocityX     = velocity.getXComponent();
        double velocityY     = velocity.getYComponent();
        double angularVelocity = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        dataRecorder.write(String.format(Locale.US,
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,"
                        + "%.4f,%.4f,%.4f,%d,%d,%d,%.4f,%.4f,%.4f\n",
                t,
                leftRear.getPower(),
                rightRear.getPower(),
                leftFront.getPower(),
                rightFront.getPower(),
                x,
                y,
                heading,
                voltage,
                driveFwd,
                driveStr,
                driveTurn,
                driveMode,
                controls.cross.justPressed() ? 1 : 0,
                controls.circle.justPressed() ? 1 : 0,
                velocityX,
                velocityY,
                angularVelocity
        ));
    }

    private void handleRecordingError(IOException error) {
        recordingError = error.getMessage() == null
                ? error.getClass().getSimpleName() : error.getMessage();
    }

    private void closeRecorder() {
        if (dataRecorder == null) {
            return;
        }

        try {
            dataRecorder.flush();
            dataRecorder.close();
        } catch (IOException e) {
            handleRecordingError(e);
        } finally {
            dataRecorder = null;
        }
    }
}
