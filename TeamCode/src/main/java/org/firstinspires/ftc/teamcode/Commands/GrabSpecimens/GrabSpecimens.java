package org.firstinspires.ftc.teamcode.Commands.GrabSpecimens;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class GrabSpecimens extends SequentialCommandGroup {

    public GrabSpecimens(Arm arm, Elevator elevator, Wrist wrist){
        addCommands(
            new MoveArm(arm, Constants.Arm.ARM_SPECIMENS).withTimeout(500),
            new MoveWrist(wrist, (0.5)).withTimeout(500),
            new ElevatorPositions(elevator,20).withTimeout(700)


        );
    }

}
