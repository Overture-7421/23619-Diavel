package org.firstinspires.ftc.teamcode.Subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;


public class Wrist extends SubsystemBase {

    // Motor Declaration
    private ServoEx wristServo;


    public Wrist (HardwareMap hardwareMap) {
        //Servos IDs
        wristServo = new SimpleServo(hardwareMap, "wristServo", 0, 1.0);
    }

    public void WristPosition(double wrist_Position) {
        wristServo.setPosition(wrist_Position);
    }

}
