package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ModifyArmCommand extends CommandBase {

    private final Arm arm;
    private Gamepad gamepad;

    private static final double INCREMENT = 2.0;

    public ModifyArmCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    // Method to set the gamepad dynamically
    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    @Override
    public void execute() {
        if (gamepad != null) {
            if (gamepad.dpad_up) {
                arm.setTarget(arm.getPosition() + INCREMENT);
            } else if (gamepad.dpad_down) {
                arm.setTarget(arm.getPosition() - INCREMENT);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
