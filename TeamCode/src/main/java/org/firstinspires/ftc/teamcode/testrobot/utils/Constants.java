package org.firstinspires.ftc.teamcode.testrobot.utils;

import com.bylazar.configurables.annotations.Configurable;

/** Constants shared by TestRobot OpModes and components. */
@Configurable
public final class Constants {
    public static final String MAIN_GROUP = "TestRobot";

    /// DRIVETRAIN CONSTANTS
    public static double CONTROLLER_DEADZONE = 0.05;
    public static double MANUAL_SPEED = 1.0;
    public static double PATH_MAX_POWER = 0.8;

    /// HEADING CONSTANTS
    public static double COMMON_BASKET_Y = 130.34;
    public static double BLUE_BASKET_X = 16.36;
    public static double RED_BASKET_X = 127.64;
    public static double AIM_KP = 1.3;
    public static double AIM_KI = 0.2;
    public static double AIM_KD = 0.1;
    public static double AIM_DEADBAND_RAD = Math.toRadians(1.5);
    public static double AIM_INTEGRAL_LIMIT = 0.5;

    /// COMPONENTS CONSTANTS
    public static double INTAKE_POWER = 0.3;

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
