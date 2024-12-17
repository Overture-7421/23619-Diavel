package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.overture.ftc.overftclib.Contollers.PIDController;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Elevator extends SubsystemBase {

    private final Telemetry telemetry;
    private final DcMotorEx elevatorMotor;
    //private ProfiledPIDController elevatorMotorPID;
    private PIDController elevatorMotorPID;
    public static final double TICKS_PER_REVOLUTION = 753.2;
    public static final double ELEVATOR_WINCH_CIRCUMFERENCE = 12.0008738;
    // In Meters diameter: 3.82 cm
    public static final double GEAR_REDUCTION = 26.9;

    public static double target = 0;
    public static double p = 0;

    public Elevator(HardwareMap hardwareMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        elevatorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "elevator_Motor");

        //elevatorMotorPID = new ProfiledPIDController(1.0,0,0, new TrapezoidProfile.Constraints(100.0,80.0));

        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //elevatorMotorPID.reset(getHeight());
        //elevatorMotorPID.setGoal(getHeight());

    }

    public double getHeight() {
        double elevatorMotorTicks = elevatorMotor.getCurrentPosition();
        return elevatorMotorTicks * ((ELEVATOR_WINCH_CIRCUMFERENCE/TICKS_PER_REVOLUTION));
    }

    public void setGoal(double goalHeight) {
        //elevatorMotorPID.reset(getHeight());
        target = goalHeight;
    }


    //Periodic actions used for positional Elevator
    @Override
    public void periodic() {
        double outputMotor = elevatorMotorPID.calculate(getHeight(), target);
        elevatorMotor.setPower(outputMotor);

        if(getHeight() > 97){
            elevatorMotor.setPower(0.0);
        }

        if(getHeight() < -0.1){
            elevatorMotor.setPower(0.0);
        }

        telemetry.addData("Elevator Pos", getHeight());
        telemetry.addData("Elevator Target", target);
    }
}
