package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.RamsetteCommand;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.TurnToAngle;
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Baskets.HighBasket;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import java.util.Arrays;


@Autonomous
public class HighBasketAndPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis = new Chassis(hardwareMap);
        Elevator elevator= new Elevator(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);


        //Forward
        TrajectoryConfig ForwardConfig = new TrajectoryConfig(1,0.2);
        ForwardConfig.setReversed(false);

        //Backward
        TrajectoryConfig ReverseConfig = new TrajectoryConfig(0.5,0.2);
        ReverseConfig.setReversed(true);

        Trajectory First = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(1.15,-0.22,Rotation2d.fromDegrees(45))), ForwardConfig
        );







        SequentialCommandGroup FirstCommandGroup = new SequentialCommandGroup(

                new RamsetteCommand(chassis, First),
                  new WaitCommand(500),
               // new HighBasket(arm, elevator).withTimeout(1500),

                //new MoveArm(arm , 100).withTimeout(500),
                new MoveArm(arm, 50).withTimeout(750),
                new ElevatorPositions(elevator,69 ).withTimeout(1500),
               new MoveIntake(intake, 0).withTimeout(500),
                new WaitCommand(500),
                new StowAll(arm, elevator),
                new WaitCommand(500),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(-100)).withTimeout(3800),

                new MoveArm(arm, -28).withTimeout(500),
                new ElevatorPositions(elevator, 56).withTimeout(1500),
                new MoveIntake(intake, 1).withTimeout(500),
                new WaitCommand(500),
                new StowAll(arm, elevator),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(45)).withTimeout(3500),
                new MoveArm(arm, 49).withTimeout(750),
                new ElevatorPositions(elevator,72 ).withTimeout(2500),
                new MoveIntake(intake, 0).withTimeout(50),
                new WaitCommand(500),
                new StowAll(arm, elevator)
        );


        waitForStart();
        chassis.reset(new Pose2d());
        CommandScheduler.getInstance().schedule(FirstCommandGroup);

        while (opModeIsActive ()){
            CommandScheduler.getInstance().run();

            Pose2d pose = chassis.getPose();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getRotation().getDegrees());

            telemetry.addLine("--- Chassis Telemetry ---");
            telemetry.addData("RightDistance", chassis.rightDistance());
            telemetry.addData("LeftDistance", chassis.leftDistance());
            telemetry.update();
        }
    }
}