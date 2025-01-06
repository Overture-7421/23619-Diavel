package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Commands.LowBasket;
import org.firstinspires.ftc.teamcode.Commands.MoveArm;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Commands.MoveIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.ElevatorPositions;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Button;
import org.firstinspires.ftc.teamcode.Commands.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.MoveIntake;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Commands.Drive;



@TeleOp
public class MainSystem extends LinearOpMode {
    @Override
    public void runOpMode(){
          
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
    telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

    /* SUBSYSTEM DECLARATION */
    Chassis chassis = new Chassis(hardwareMap);
    Intake intake = new Intake(hardwareMap);
    Arm arm = new Arm(hardwareMap);
    Elevator elevator = new Elevator(hardwareMap);

    /* GAMEPAD DECLARATION */
    GamepadEx driver = new GamepadEx(gamepad1);
    GamepadEx operator = new GamepadEx(gamepad2);

    /* COMMAND DECLARATION */
        // CHASSIS
        chassis.setDefaultCommand(new Drive(chassis,gamepad1));

        // INTAKE
        Button driverButtonX= driver.getGamepadButton(GamepadKeys.Button.X);
        driverButtonX.whenHeld(new MoveIntake(intake,1.0));
        driverButtonX.whenReleased(new MoveIntake(intake,0.0));

        Button driverButtonB= driver.getGamepadButton(GamepadKeys.Button.B);
        driverButtonB.whenHeld(new MoveIntake(intake,-1.0));
        driverButtonB.whenReleased(new MoveIntake(intake,0.0));

        // ARM
        Button operatorButtonA= operator.getGamepadButton(GamepadKeys.Button.A);
        operatorButtonA.whenPressed(new MoveArm(arm,90.0));

        Button operatorButtonY= operator.getGamepadButton(GamepadKeys.Button.Y);
        operatorButtonY.whenPressed(new MoveArm(arm,0.0));

        // ELEVATOR
        Button operatorButtonX= operator.getGamepadButton(GamepadKeys.Button.X);
        operatorButtonX.whenPressed(new ElevatorPositions(elevator,60.0));

        Button operatorButtonDPADUP= operator.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        operatorButtonDPADUP.whenPressed(new ElevatorPositions(elevator,60.0));

        Button operatorButtonDPADDOWN= operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        operatorButtonDPADDOWN.whenPressed(new ElevatorPositions(elevator,0.0));

        // GAME ROUTINES
        //Button operatorButtonA=operator.getGamepadButton(GamepadKeys.Button.A);
        //operatorButtonA.whenPressed(new BasketPos(arm, elevator, Constants.Arm.ARMHIGHCHAMBER, Constants.Elevator.ELEVATORHIGHCHAMBER));

        //Button operatorButtonDPAD= operator.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        //operatorButtonDPAD.whenPressed(new LowBasket(elevator arm));

        waitForStart();
            chassis.reset(new Pose2d(0,0, Rotation2d.fromDegrees(0)));

            while (opModeIsActive()) {
                CommandScheduler.getInstance().run();
                Pose2d pose = chassis.getPose();

                // -- ODOMETRY TELEMETRY -- //
                    telemetry.addData("X", pose.getX());
                    telemetry.addData("Y", pose.getY());
                    telemetry.addData("Heading", pose.getRotation().getDegrees());
                    telemetry.addData("RightDistance", chassis.rightDistance());
                    telemetry.addData("LeftDistance", chassis.leftDistance());
                    telemetry.addData("Elevator_Distance", elevator.getHeight());

                // -- UPDATE TELEMETRY -- //
                    telemetry.update();

            }
    }
}
