package org.firstinspires.ftc.teamcode.Commands.GrabSpecimens;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class GrabSpecimens extends SequentialCommandGroup {

    public GrabSpecimens(Arm arm, Elevator elevator){
        addCommands(
            new MoveArm(arm, Constants.Arm.ARM_CLIMB).withTimeout(500),
            new ElevatorPositions(elevator,20)


        );
    }

}
