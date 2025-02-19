package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.Commands.Arm.ModifyArmCommand;
import org.firstinspires.ftc.teamcode.Commands.Baskets.HighBasket;
import org.firstinspires.ftc.teamcode.Commands.Chambers.HighChamber;
import org.firstinspires.ftc.teamcode.Commands.Baskets.LowBasket;
import org.firstinspires.ftc.teamcode.Commands.Chambers.LowChamber;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ModifyElevatorCommand;
import org.firstinspires.ftc.teamcode.Commands.GrabSpecimens.GrabSpecimens;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.GroundGrabHover;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.GroundGrabPick;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Button;

import org.firstinspires.ftc.teamcode.Commands.Drive;


@TeleOp
public class MainSystem extends LinearOpMode {

    private ModifyArmCommand modifyArmCommand;
    private ModifyElevatorCommand modifyElevatorCommand;

    @Override
    public void runOpMode(){
    CommandScheduler.getInstance().reset();

    /* SUBSYSTEM DECLARATION */
    Chassis chassis = new Chassis(hardwareMap);
    Intake intake = new Intake(hardwareMap);
    Arm arm = new Arm(hardwareMap);
    Elevator elevator = new Elevator(hardwareMap);
    Wrist wrist = new Wrist(hardwareMap);

    /* GAMEPAD DECLARATION */
    GamepadEx driver = new GamepadEx(gamepad1);
    GamepadEx operator = new GamepadEx(gamepad2);

    /* COMMAND DECLARATION */
        // CHASSIS
        chassis.setDefaultCommand(new Drive(chassis,gamepad1));

        // MANUAL INTAKE
        Button driverButtonX= driver.getGamepadButton(GamepadKeys.Button.X);
        driverButtonX.whenPressed(new MoveIntake(intake,1.0));

        Button driverButtonB= driver.getGamepadButton(GamepadKeys.Button.B);
        driverButtonB.whenPressed(new MoveIntake(intake,0.0));


        // GROUND GRAB COMMANDS
            // GROUND GRAB
            Button driverRightBumper = driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            driverRightBumper.whenPressed(new GroundGrabHover(arm, elevator, wrist, intake, driver));

            Button driverButtonA = driver.getGamepadButton(GamepadKeys.Button.A);
            driverButtonA.whenPressed(new GroundGrabPick(intake, arm));


        // WRIST MANUAL POSITIONS AND TEST
            Button driverRightJoystickButton = driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
            driverRightJoystickButton.whenPressed(new MoveWrist(wrist, Constants.Wrist.WRIST_SHORT));

            Button driverLeftJoystickButton = driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
            driverLeftJoystickButton.whenPressed(new MoveWrist(wrist, Constants.Wrist.WRIST_LONG));

        /* !!! To employ please comment the usage of button A, Y and RIGHTBUMPER for the driver in GroundGrab and uncomment this !!! */
                //driverButtonA.whenPressed(new MoveWrist(wrist, Constants.Wrist.WRIST_SHORT));
                //driverButtonY.whenPressed(new MoveWrist(wrist, Constants.Wrist.WRIST_MEDIUM));
                //driverRightBumper.whenPressed(new MoveWrist(wrist, Constants.Wrist.WRIST_LONG));


        // MANUAL ARM
        modifyArmCommand = new ModifyArmCommand(arm);
        modifyArmCommand.setGamepad(gamepad1);
        arm.setDefaultCommand(modifyArmCommand);

        // MANUAL ELEVATOR
        modifyElevatorCommand = new ModifyElevatorCommand(elevator);
        modifyElevatorCommand.setGamepad(gamepad1);
        elevator.setDefaultCommand(modifyElevatorCommand);

        /* GAME ROUTINES */
            // BASKETS
            Button operatorButtonA = operator.getGamepadButton(GamepadKeys.Button.A);
            operatorButtonA.whileHeld(new LowBasket(arm, elevator, wrist));
            operatorButtonA.whenReleased(new StowAll(arm, elevator));

            Button operatorButtonB = operator.getGamepadButton(GamepadKeys.Button.B);
            operatorButtonB.whileHeld(new HighBasket( arm, elevator, wrist));
            operatorButtonB.whenReleased(new HighBasket( arm, elevator, wrist));


        // CHAMBERS
            Button operatorButtonY = operator.getGamepadButton(GamepadKeys.Button.Y);
            operatorButtonY.whenPressed(new HighChamber(arm, elevator, wrist));

            Button operatorButtonX = operator.getGamepadButton(GamepadKeys.Button.X);
            operatorButtonX.whenPressed(new LowChamber(arm, elevator));

            // STOW ALL
            Button operatorRightBumper = operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            operatorRightBumper.whenPressed(new StowAll(arm, elevator));

            // CLIMB
            Button driverLeftBumper = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
            driverLeftBumper.whenPressed(new GrabSpecimens(arm, elevator, wrist));


        waitForStart();
            chassis.reset(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
            while (opModeIsActive()) {
                CommandScheduler.getInstance().run();
                Pose2d pose = chassis.getPose();

                // -- ODOMETRY TELEMETRY -- //
                    telemetry.addLine("--- IMU Telemetry ---");
                    telemetry.addData("X", pose.getX());
                    telemetry.addData("Y", pose.getY());
                    telemetry.addData("Heading", pose.getRotation().getDegrees());

                    telemetry.addLine("--- Chassis Telemetry ---");
                    telemetry.addData("RightDistance", chassis.rightDistance());
                    telemetry.addData("LeftDistance", chassis.leftDistance());

                    telemetry.addLine("--- Subsystem Telemetry ---");
                    telemetry.addData("Elevator_Distance", elevator.getHeight());
                    telemetry.addData("Arm Position", arm.getPosition());
                    telemetry.addData("Arm Target", arm.target);
                    telemetry.addData("Elevator Target", elevator.target);
                    telemetry.addData("PushButton State", arm.ActiveButtonReset);
                    telemetry.addData("PushButton", arm.pushButton.getState());

                // -- UPDATE TELEMETRY -- //
                    telemetry.update();

            }
    }
}
