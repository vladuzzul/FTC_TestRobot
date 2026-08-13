package org.firstinspires.ftc.teamcode.testrobot.utils;

/** Constants shared by TestRobot OpModes and components. */
public final class Constants {
    public static final String MAIN_GROUP = "TestRobot";
    public static final String TEST_GROUP = "TestRobot Tests";

    public static double CONTROLLER_DEADZONE = 0.05;
    public static double MANUAL_SPEED = 1.0;
    public static double PATH_MAX_POWER = 0.6;

    private Constants() {
    }

    public static double applyDeadzone(double value) {
        double magnitude = Math.abs(value);
        if (magnitude <= CONTROLLER_DEADZONE) {
            return 0.0;
        }

        double normalized = (magnitude - CONTROLLER_DEADZONE)
                / (1.0 - CONTROLLER_DEADZONE);
        return Math.copySign(normalized * MANUAL_SPEED, value);
    }
}
