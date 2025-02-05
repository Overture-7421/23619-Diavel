package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class WaitForButton extends CommandBase {

    private final GamepadEx gamepad;
    private final int buttonNumber;

    public WaitForButton(GamepadEx gamepad, int buttonNumber) {
        this.gamepad = gamepad;
        this.buttonNumber = buttonNumber;
    }

    @Override
    public boolean isFinished() {
        switch (buttonNumber) {
            case 1:
                return gamepad.getButton(GamepadKeys.Button.A);
            case 2:
                return gamepad.getButton(GamepadKeys.Button.B);
            case 3:
                return gamepad.getButton(GamepadKeys.Button.X);
            case 4:
                return gamepad.getButton(GamepadKeys.Button.Y);
            case 5:
                return gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER);
            case 6:
                return gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER);
            case 7:
                return gamepad.getButton(GamepadKeys.Button.DPAD_UP);
            case 8:
                return gamepad.getButton(GamepadKeys.Button.DPAD_DOWN);
            case 9:
                return gamepad.getButton(GamepadKeys.Button.DPAD_LEFT);
            case 10:
                return gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT);
            case 11:
                return gamepad.getButton(GamepadKeys.Button.START);
            case 12:
                return gamepad.getButton(GamepadKeys.Button.BACK);
            default:
                throw new IllegalArgumentException("Invalid button number: " + buttonNumber);
        }
    }
}
