package org.firstinspires.ftc.teamcode.testrobot.utils;

/** Constants shared by TestRobot OpModes and components. */
public final class Constants {
    public static final String MAIN_GROUP = "TestRobot";

    public static double CONTROLLER_DEADZONE = 0.05;
    public static double MANUAL_SPEED = 1.0;
    public static double PATH_MAX_POWER = 1;

    public static double COMMON_BASKET_Y = 130.34;
    public static double BLUE_BASKET_X = 16.36;
    public static double RED_BASKET_X = 127.64;

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
