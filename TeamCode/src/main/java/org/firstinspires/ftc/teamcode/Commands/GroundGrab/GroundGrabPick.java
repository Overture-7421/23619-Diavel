package org.firstinspires.ftc.teamcode.Commands.GroundGrab;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class GroundGrabPick extends SequentialCommandGroup   {
    public GroundGrabPick(Intake intake, Arm arm){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_GROUNDGRAB_PICK).withTimeout(100),
                new MoveIntake(intake, Constants.Intake.INTAKE_STOW).withTimeout(100),
                new WaitCommand(500),
                new MoveArm(arm, Constants.Arm.ARM_GROUNDGRAB).withTimeout(250)
        );
    }
}