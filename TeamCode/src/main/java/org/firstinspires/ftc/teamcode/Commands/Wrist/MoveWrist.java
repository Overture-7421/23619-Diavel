package org.firstinspires.ftc.teamcode.Commands.Wrist;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import java.util.concurrent.TimeUnit;

public class MoveWrist extends CommandBase {
    private Wrist wrist;
    private double WristMotorPosition;
    private Timing.Timer timer;

    public MoveWrist(Wrist subsystem, double WristMotorPosition) {
        this.WristMotorPosition = WristMotorPosition;
        wrist = subsystem;
        timer = new Timing.Timer(1, TimeUnit.SECONDS);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        wrist.WristPosition(WristMotorPosition);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
