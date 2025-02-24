package org.firstinspires.ftc.teamcode.Commands.GroundGrab;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.WaitForButton;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


public class GroundGrabHover extends SequentialCommandGroup {

    public GroundGrabHover(Arm arm, Elevator elevator, Wrist wrist, Intake intake, GamepadEx driver){
        addCommands(
                //new WaitForButton(driver, GamepadKeys.Button.RIGHT_BUMPER),
                new MoveArm(arm, Constants.Arm.ARM_GROUNDGRAB).withTimeout(500),
                new MoveIntake(intake, Constants.Intake.INTAKE_OPEN).withTimeout(500),

                //new WaitForButton(driver, GamepadKeys.Button.Y),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_HOVER_GROUNDGRAB).withTimeout(1500),
                new MoveWrist(wrist, Constants.Wrist.WRIST_GROUNDGRAB).withTimeout(500)
        );
    }

}