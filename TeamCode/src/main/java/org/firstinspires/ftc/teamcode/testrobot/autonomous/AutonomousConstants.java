package org.firstinspires.ftc.teamcode.testrobot.autonomous;

import com.pedropathing.geometry.Pose;

/** Field coordinates used by autonomous and assisted TeleOp movement. */
public final class AutonomousConstants {
    public static double LEFT_START_X = 9.0;
    public static double LEFT_START_Y = 9.0;
    public static double LEFT_START_HEADING_DEG = 90.0;

    public static double RIGHT_START_X = 135.0;
    public static double RIGHT_START_Y = 9.0;
    public static double RIGHT_START_HEADING_DEG = 90.0;

    public static double CENTER_X = 72.0;
    public static double CENTER_Y = 72.0;
    public static double CENTER_HEADING_DEG = 90.0;

    private AutonomousConstants() {
    }

    public static Pose leftStartPose() {
        return pose(LEFT_START_X, LEFT_START_Y, LEFT_START_HEADING_DEG);
    }

    public static Pose rightStartPose() {
        return pose(RIGHT_START_X, RIGHT_START_Y, RIGHT_START_HEADING_DEG);
    }

    public static Pose centerPose() {
        return pose(CENTER_X, CENTER_Y, CENTER_HEADING_DEG);
    }

    private static Pose pose(double x, double y, double headingDegrees) {
        return new Pose(x, y, Math.toRadians(headingDegrees));
    }
}
