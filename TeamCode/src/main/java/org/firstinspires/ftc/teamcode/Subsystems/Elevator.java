package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.overture.ftc.overftclib.Contollers.PIDController;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;

import java.io.PushbackInputStream;

//@Config
public class Elevator extends SubsystemBase {
//public static double d;
    //public static double p;
    //private final Telemetry telemetry;



    private final DcMotorEx elevatorMotor;
    private PIDController elevatorMotorPID;
    private final double TICKS_PER_REVOLUTION = 753.2;
    private  final double ELEVATOR_WINCH_CIRCUMFERENCE = 12.0008738;
    // In Meters diameter: 3.82 cm
    private final double GEAR_REDUCTION = 26.9;

    public double ActiveBottonReset=0;

    public final DigitalChannel elevatorBotton;

    public  double target = 0;

    public Elevator(HardwareMap hardwareMap) {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();
        elevatorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "elevator_Motor");

        elevatorMotorPID = new PIDController(0.6,0,0.025);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorBotton = hardwareMap.get(DigitalChannel .class, "elevator_touch");
        elevatorBotton.setMode(DigitalChannel.Mode.INPUT);


      }

    public double getHeight() {
        double elevatorMotorTicks = elevatorMotor.getCurrentPosition();
        return elevatorMotorTicks * ((ELEVATOR_WINCH_CIRCUMFERENCE/TICKS_PER_REVOLUTION));
    }

    public void setGoal(double goalHeight) {
        target = goalHeight;
    }


    //Periodic actions used for positional Elevator
    @Override
    public void periodic() {
        //elevatorMotorPID.setD(d);
        // elevatorMotorPID.setP(p);
        double outputMotor = elevatorMotorPID.calculate(getHeight(), target);
        elevatorMotor.setPower(outputMotor);

        if (elevatorBotton.getState()==true  && ActiveBottonReset == 0){
            elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ActiveBottonReset= 1;
            elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setGoal(0);
        } else {
            if (ActiveBottonReset==1 && elevatorBotton.getState() == false){
                ActiveBottonReset = 0;

                elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


        }




        /*if(getHeight() > 97){
            elevatorMotor.setPower(0.0);
        }

        if(getHeight() < 0.1){
            elevatorMotor.setPower(0.0);
        }*/
       /* if(target < 0){
            target= 0;
        }*/

        //telemetry.addData("Elevator Height", getHeight());
        //telemetry.addData("Elevator Target", target);
    }


}
