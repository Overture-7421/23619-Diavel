package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;


import com.overture.ftc.overftclib.Contollers.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* FOR DASHBOARD USE ONLY
* Configuration variables are special fields that the dashboard client can seamlessly modify while the app is running.
* To mark a field as a config variable, declare it static and not final and annotate the enclosing class with @Config.
* */

//@Config
public class Arm extends SubsystemBase {

    TouchSensor arm_touch;

    private final DcMotorEx motor;
    private final PIDController armPID;
    //private final Telemetry telemetry;

    private static final double COUNTS_PER_REV = 8192;
    private static final double OFFSET = 47;
    private double target = -47;
    private static final double ff = 0.180;


    public Arm(HardwareMap hardwareMap) {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();
        motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm_Motor");
        arm_touch = hardwareMap.get(TouchSensor.class, "arm_touch");

        armPID = new PIDController(0.044, 0.0, 0.0);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double armFeedForward(double angle){
        return ((Math.cos(Math.toRadians(angle))) * ff);

    }

    public double getPosition() {
        double currentTicks = motor.getCurrentPosition();
        //return -((currentTicks / COUNTS_PER_REV) * 360 + OFFSET);
        return ((currentTicks / COUNTS_PER_REV) * 360 - OFFSET);
    }


    public void setTarget(double targetPos) {
        target = targetPos;

    }

    public boolean isTouchPressed() {
        return arm_touch.isPressed();
    }

    @Override
    public void periodic() {
        // Check if the touch sensor is pressed
        if (arm_touch.isPressed()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            target = -OFFSET;
        }

        double motorOutput = armPID.calculate(getPosition(), target);
        motor.setPower(motorOutput + armFeedForward(getPosition()));

        /*telemetry.addData("Arm Output", motorOutput);
        telemetry.addData("Arm Position", getPosition());
        telemetry.addData("Arm Target", target);*/


    }
}