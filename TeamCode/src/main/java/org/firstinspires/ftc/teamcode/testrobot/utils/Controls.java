package org.firstinspires.ftc.teamcode.testrobot.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

/** Snapshot of driver inputs with rising-edge button detection. */
public final class Controls {
    private final Gamepad gamepad;

    public final Button a = new Button();
    public final Button b = new Button();
    public final Button dpadLeft = new Button();
    public final Button dpadRight = new Button();
    public final Button dpadDown = new Button();
    public final Button dpadUp = new Button();
    public final Button leftStickButton = new Button();


    public double leftStickX;
    public double leftStickY;
    public double rightStickX;

    public Controls(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        a.update(gamepad.a);
        b.update(gamepad.b);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);
        dpadDown.update(gamepad.dpad_down);
        dpadUp.update(gamepad.dpad_up);
        leftStickButton.update(gamepad.left_stick_button);

        leftStickX = gamepad.left_stick_x;
        leftStickY = gamepad.left_stick_y;
        rightStickX = gamepad.right_stick_x;
    }

    public static final class Button {
        private boolean previous;
        private boolean current;

        private void update(boolean value) {
            previous = current;
            current = value;
        }

        public boolean pressed() {
            return current;
        }

        public boolean justPressed() {
            return current && !previous;
        }
    }
}
