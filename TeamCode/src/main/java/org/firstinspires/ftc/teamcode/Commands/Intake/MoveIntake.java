package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import java.util.concurrent.TimeUnit;

public class MoveIntake extends CommandBase {
    private Intake intake;
    private double IntakeMotorPosition;
    private Timing.Timer timer;

    public MoveIntake(Intake subsystem, double IntakeMotorPosition) {
        this.IntakeMotorPosition = IntakeMotorPosition;
        intake = subsystem;
        timer = new Timing.Timer(1, TimeUnit.SECONDS);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intake.IntakePosition(IntakeMotorPosition);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
