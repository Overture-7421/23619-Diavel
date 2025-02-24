package org.firstinspires.ftc.teamcode.Commands.Chambers;
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;


public class HighChamber extends SequentialCommandGroup {

    public HighChamber (Arm arm, Elevator elevator, Wrist wrist){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_HIGHCHAMBER).withTimeout(500),
                new MoveWrist(wrist, 0.5).withTimeout(500),
                new ElevatorPositions(elevator,Constants.Elevator.ELEVATOR_HIGHCHAMBER).withTimeout(700),
                new WaitCommand(1500),
                new MoveArm(arm, Constants.Arm.ARM_HIGHCHAMBER-15).withTimeout(500)
                //new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_STOW),
                //new MoveArm(arm, Constants.Arm.ARM_STOW)



                //new MoveArm(arm, Constants.Arm.ARM_HIGHCHAMBER-15).withTimeout(500),
                //new WaitCommand(1000),
                //new StowAll(arm, elevator)
        );


    }

}
