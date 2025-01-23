package org.firstinspires.ftc.teamcode.Commands.Climber;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class Climb extends SequentialCommandGroup {

    public Climb (Arm arm, Elevator elevator){
        addCommands(
            new MoveArm(arm, Constants.Arm.ARM_CLIMB).withTimeout(500),
            new ElevatorPositions(elevator,20),
            new WaitCommand(4000),

                new ElevatorPositions(elevator, 0 ).withTimeout(500),
            new MoveArm(arm, 0).withTimeout(500)
        );
    }

}
