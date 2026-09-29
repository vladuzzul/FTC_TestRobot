package org.firstinspires.ftc.teamcode.testrobot.opmodes.auto;

import android.os.Environment;
import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.teamcode.testrobot.Robot;
import org.firstinspires.ftc.teamcode.testrobot.commands.ReplayCommand;
import org.firstinspires.ftc.teamcode.testrobot.opmodes.base.RobotOpMode;
import org.firstinspires.ftc.teamcode.testrobot.utils.RecordingFormat;
import java.io.File;
import java.io.IOException;

@NextAutonomous(name = "Replay", group = "Replay", preselectTeleop = "Main Driving")
public final class ReplayOp extends RobotOpMode {
    private ReplayCommand replay;
    private String error;
    private File recording;

    public ReplayOp(Robot robot) { super(robot); }

    @Override protected void initialize() {
        recording = new File(Environment.getExternalStorageDirectory(), RecordingFormat.FILE_NAME);
        try {
            replay = new ReplayCommand(recording, robot.drive, hardwareMap, telemetry);
        } catch (IOException e) {
            error = e.getMessage();
            return;
        }
        robot.initialize(hardwareMap, replay.startingPose());
    }

    @Override public void disabledPeriodic() {
        telemetry.addData("Replay file", recording.getAbsolutePath());
        if (error != null) telemetry.addData("Replay error", error);
        else {
            telemetry.addData("Frames", replay.frameCount());
            telemetry.addData("Duration", "%.2f s", replay.duration());
            telemetry.addLine("Ready to replay");
        }
    }

    @Override protected void onStart() {
        if (replay != null) replay.schedule();
    }

    @Override public void periodic() {
        if (error != null) telemetry.addData("Replay error", error);
        else if (!replay.isScheduled()) telemetry.addLine("Replay complete");
    }

    @Override protected void beforeShutdown() {
        if (replay != null) replay.cancel();
    }
}
