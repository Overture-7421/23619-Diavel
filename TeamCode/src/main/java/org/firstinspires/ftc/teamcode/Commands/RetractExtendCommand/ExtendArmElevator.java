package org.firstinspires.ftc.teamcode.Commands.RetractExtendCommand;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.MainSystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.Index;

public class ExtendArmElevator extends SequentialCommandGroup  {

public PositionsTable positionsTable;

    public ExtendArmElevator (Arm arm, Elevator elevator, PositionsTable positionsTable){
        Index.setIndex(Index.getIndexValue()+1);
        addCommands(

               new MoveArm(arm, positionsTable.getArmTarget(Index.getIndexValue())).withTimeout(300),
               new ElevatorPositions(elevator, positionsTable.getElevatorTarget(Index.getIndexValue())).withTimeout(300)


        );
    }

}

// PENDING