package org.firstinspires.ftc.teamcode.Subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;


public class Intake extends SubsystemBase {

    // Motor Declaration
    private ServoEx intakeServo;


    public Intake (HardwareMap hardwareMap) {
        //Servos IDs
        intakeServo = new SimpleServo(hardwareMap, "grabServo", 0, 1.0);
    }

    public void IntakePosition(double intake_Position) {
        intakeServo.setPosition(intake_Position);
    }

}
