package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;


package org.firstinspires.ftc.teamcode.Commands;
import android.text.style.BackgroundColorSpan;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Commands.MoveArm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class LowChamber extends SequentialCommandGroup {

    public void LowChamber (Arm arm, Elevator elevator){
        addCommands(

                new MoveArm(arm, 1.0),
                new WaitCommand(500),
                new MoveElevator(elevator, 1.0)

        );


    }

}