package org.firstinspires.ftc.teamcode.testrobot.utils;

/** Shared contract between RecorderOp and ReplayOp. */
public final class RecordingFormat {
    public static final String FILE_NAME = "robot_recording.csv";
    public static final String HEADER =
            "Time,LR,RR,LF,RF,X,Y,Heading,Voltage,"
                    + "DriveFwd,DriveStr,DriveTurn,DriveMode,"
                    + "APressed,BPressed,VelX,VelY,AngularVelocity,"
                    + "FieldCentric,AimingAtTarget,AimingAtBlue";
    public static final int COLUMN_COUNT = 21;

    public static final int MODE_MANUAL = 0;
    public static final int MODE_FOLLOWING_PATH = 1;
    public static final int MODE_HOLDING_POSE = 2;

    private RecordingFormat() {
    }
}
