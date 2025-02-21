package org.firstinspires.ftc.teamcode.Commands.Chambers;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Commands.WaitForButton;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class LowChamber extends SequentialCommandGroup {

    public LowChamber (Arm arm, Elevator elevator, Wrist wrist, GamepadEx operator){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_LOWCHAMBER).withTimeout(500),
                new WaitForButton(operator, GamepadKeys.Button.X),

                new MoveArm(arm, -15).withTimeout(500),
                new WaitForButton(operator, GamepadKeys.Button.X),

                new StowAll(arm, elevator, wrist)

        );


    }

}