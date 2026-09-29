package org.firstinspires.ftc.teamcode.testrobot.utils;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.Gamepad;

public final class GamepadEffects {
    private static final int RGB_STEPS = 24;
    private static final int RGB_STEP_MS = 75;
    private static final Gamepad.LedEffect RGB_EFFECT = buildRgbEffect();

    private GamepadEffects() {}

    public static void startRgbEffect(Gamepad gamepad) {
        gamepad.runLedEffect(RGB_EFFECT);
    }

    private static Gamepad.LedEffect buildRgbEffect() {
        Gamepad.LedEffect.Builder builder = new Gamepad.LedEffect.Builder()
                .setRepeating(true);

        for (int i = 0; i < RGB_STEPS; i++) {
            float hue = 360f * i / RGB_STEPS;
            int color = Color.HSVToColor(new float[]{hue, 1f, 1f});

            builder.addStep(
                    Color.red(color) / 255.0,
                    Color.green(color) / 255.0,
                    Color.blue(color) / 255.0,
                    RGB_STEP_MS);
        }

        return builder.build();
    }

}
