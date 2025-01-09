package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class ExtendArmElevator extends SequentialCommandGroup {
    public void Extend (Arm arm, Elevator elevator, PositionsTable positionsTable){
        addCommands(
                new MoveArm(arm, positionsTable.getArmTarget(positionsTable.Index+1)).withTimeout(1000),
                new ElevatorPositions(elevator, positionsTable.getElevatorTarget(positionsTable.Index+1)).withTimeout(1000)
        );
    }
}
