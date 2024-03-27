package org.firstinspires.ftc.teamcode.util.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class StatefulGamepad {
    private final Gamepad gamepad;

    private final HashMap<GamepadButton, Boolean> previousState = new HashMap<>();
    private final HashMap<GamepadButton, Boolean> currentState = new HashMap<>();

    public StatefulGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
        for (GamepadButton button : GamepadButton.values()) {
            previousState.put(button, getButton(button));
            currentState.put(button, getButton(button));
        }
    }

    public void update() {
        for (GamepadButton button : GamepadButton.values()) {
            previousState.put(button, currentState.get(button));
            currentState.put(button, getButton(button));
        }
    }

    public boolean wasJustPressed(GamepadButton button) {
        return getCurrentState(button) && !getPreviousState(button);
    }

    public boolean wasJustReleased(GamepadButton button) {
        return !getCurrentState(button) && getPreviousState(button);
    }

    public boolean stateJustChanged(GamepadButton button) {
        return getCurrentState(button) != getPreviousState(button);
    }

    private boolean getCurrentState(GamepadButton button) {
        return Boolean.TRUE.equals(currentState.get(button));
    }

    private boolean getPreviousState(GamepadButton button) {
        return Boolean.TRUE.equals(previousState.get(button));
    }

    public boolean getButton(GamepadButton button) {
        switch (button) {
            case A:
                return gamepad.a;
            case B:
                return gamepad.b;
            case X:
                return gamepad.x;
            case Y:
                return gamepad.y;
            case DPAD_UP:
                return gamepad.dpad_up;
            case DPAD_DOWN:
                return gamepad.dpad_down;
            case DPAD_LEFT:
                return gamepad.dpad_left;
            case DPAD_RIGHT:
                return gamepad.dpad_right;
            case LEFT_BUMPER:
                return gamepad.left_bumper;
            case RIGHT_BUMPER:
                return gamepad.right_bumper;
            case LEFT_STICK_BUTTON:
                return gamepad.left_stick_button;
            case RIGHT_STICK_BUTTON:
                return gamepad.right_stick_button;
            default:
                return false;
        }
    }
}
