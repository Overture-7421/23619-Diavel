package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Commands.Arm.ModifyArmCommand;
import org.firstinspires.ftc.teamcode.Commands.Baskets.HighBasket;
import org.firstinspires.ftc.teamcode.Commands.Chambers.HighChamber;
import org.firstinspires.ftc.teamcode.Commands.Baskets.LowBasket;
import org.firstinspires.ftc.teamcode.Commands.Chambers.LowChamber;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ModifyElevatorCommand;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab;
import org.firstinspires.ftc.teamcode.Commands.RetractExtendCommand.ExtendArmElevator;
import org.firstinspires.ftc.teamcode.Commands.RetractExtendCommand.RetractArmElevator;
import org.firstinspires.ftc.teamcode.Commands.RetractExtendCommand.PositionsTable;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
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

    PositionsTable positionsTable;

    @Override
    public void runOpMode(){
    //CommandScheduler.getInstance().cancelAll();
    //CommandScheduler.getInstance().reset();
    //telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

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

        // MANUAL INTAKE
        Button driverButtonX= driver.getGamepadButton(GamepadKeys.Button.X);
        driverButtonX.whenHeld(new MoveIntake(intake,1.0));
        driverButtonX.whenReleased(new MoveIntake(intake,0.0));

        Button driverButtonB= driver.getGamepadButton(GamepadKeys.Button.B);
        driverButtonB.whenHeld(new MoveIntake(intake,-1.0));
        driverButtonB.whenReleased(new MoveIntake(intake,0.0));

        Button driverButtonRigtBumper= driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        driverButtonRigtBumper.whenHeld(new GroundGrab(arm, elevator));
        driverButtonRigtBumper.whenReleased(new GroundGrab(arm, elevator));

        // MANUAL ARM
        modifyArmCommand = new ModifyArmCommand(arm, gamepad1);
        arm.setDefaultCommand(modifyArmCommand);

        // MANUAL ELEVATOR
        modifyElevatorCommand = new ModifyElevatorCommand(elevator, gamepad1);
        elevator.setDefaultCommand(modifyElevatorCommand);

        // GAME ROUTINES
            // BASKETS
            Button operatorButtonA = operator.getGamepadButton(GamepadKeys.Button.A);
            operatorButtonA.whenPressed(new LowBasket(arm, elevator));

            Button operatorButtonB = operator.getGamepadButton(GamepadKeys.Button.B);
            operatorButtonB.whenPressed(new HighBasket(intake, arm, elevator));

            // CHAMBERS
            Button operatorButtonY = operator.getGamepadButton(GamepadKeys.Button.Y);
            operatorButtonY.whenPressed(new HighChamber(arm, elevator));

            Button operatorButtonX = operator.getGamepadButton(GamepadKeys.Button.X);
            operatorButtonX.whenPressed(new LowChamber(arm, elevator));

            // STOW ALL
            Button operatorRightBumper = operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            operatorRightBumper.whenPressed(new StowAll(arm, elevator));

            // RETRACT EXTEND COMMAND
            Button operatorDPadUp = operator.getGamepadButton(GamepadKeys.Button.DPAD_UP);
            operatorDPadUp.whenPressed(new ExtendArmElevator(arm, elevator, positionsTable ));

            Button operatorDPadDown = operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
            operatorDPadDown.whenPressed(new RetractArmElevator(arm, elevator, positionsTable ));

        waitForStart();
            chassis.reset(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
            while (opModeIsActive()) {
                CommandScheduler.getInstance().run();
                Pose2d pose = chassis.getPose();

                /*if (arm.isTouchPressed()){
                    telemetry.addData("Reset Encoder", "Reset");
                } else {
                    telemetry.addData("Reset Encoder", "Not Reset");
                }*/


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

                // -- UPDATE TELEMETRY -- //
                    telemetry.update();

            }
    }
}
