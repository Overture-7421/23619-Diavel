package org.firstinspires.ftc.teamcode.Commands.RetractExtendCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.Index;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.RetractExtendCommand.PositionsTable;

public class RetractArmElevator extends SequentialCommandGroup {
    public RetractArmElevator (Arm arm, Elevator elevator, PositionsTable positionsTable){
        Index.setIndex(Index.getIndexValue()-1);
        addCommands(


                new MoveArm(arm, positionsTable.getElevatorTarget(Index.getIndexValue())).withTimeout(1000),
                new WaitCommand(500),
                new ElevatorPositions(elevator, positionsTable.getArmTarget(Index.getIndexValue())).withTimeout(1000)
              );
    }
}