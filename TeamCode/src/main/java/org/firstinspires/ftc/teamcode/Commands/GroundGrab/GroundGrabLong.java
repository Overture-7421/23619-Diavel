package org.firstinspires.ftc.teamcode.Commands.GroundGrab;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.WaitForButton;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;


public class GroundGrabLong extends SequentialCommandGroup {

    public GroundGrabLong(Arm arm, Elevator elevator, GamepadEx gamepad) {
        addCommands(
                // Wait for the right bumper to be pressed (button number 6)
                new WaitForButton(gamepad, 6),

                // Execute elevator movement after button press
                new ElevatorPositions(elevator, 10).withTimeout(500),

                // Wait for the A button to be pressed (button number 1)
                new WaitForButton(gamepad, 1),

                // Execute arm and elevator movements
                new MoveArm(arm, Constants.Arm.ARM_LONG_GROUNDGRAB).withTimeout(500),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_LONG_GROUNDGRAB).withTimeout(1500)
        );
    }
}
