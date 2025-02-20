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
import org.firstinspires.ftc.teamcode.Commands.Chambers.HighChamber;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import java.util.Arrays;


@Autonomous
public class HighChamberAndBasket extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis = new Chassis(hardwareMap);
        Elevator elevator= new Elevator(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        //Forward
        TrajectoryConfig ForwardConfig = new TrajectoryConfig(0.5,0.2);
        ForwardConfig.setReversed(false);

        //Backward
        TrajectoryConfig ReverseConfig = new TrajectoryConfig(0.5,0.2);
        ReverseConfig.setReversed(true);



        Trajectory First = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(0.8,0.,Rotation2d.fromDegrees(0))), ForwardConfig
        );

        Trajectory Middle = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.8,0,Rotation2d.fromDegrees(0)),
                new Pose2d(0.6,0.,Rotation2d.fromDegrees(0))), ForwardConfig
        );

        Trajectory Second = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.6,0,Rotation2d.fromDegrees(0)),
                new Pose2d(0.6,1,Rotation2d.fromDegrees(-90))), ReverseConfig
        );

        Trajectory Third = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.6,1,Rotation2d.fromDegrees(0)),
                new Pose2d(0.4,1,Rotation2d.fromDegrees(-45))), ReverseConfig
        );


        Trajectory Four = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.4,1,Rotation2d.fromDegrees(-90)),
                new Pose2d(0.2,-1.3,Rotation2d.fromDegrees(-90))), ReverseConfig
        );

        SequentialCommandGroup FirstCommandGroup = new SequentialCommandGroup(
                //HighChamber
                new MoveArm(arm, 40),
                new RamsetteCommand(chassis, First),
                new WaitCommand(500),
                //deja la pieza, hace falta el movimiento
                //se traslada
                new RamsetteCommand(chassis, Middle),
                new StowAll(arm, elevator, wrist),
                new RamsetteCommand(chassis, Second),

                //agarra la pieza
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new MoveArm(arm, -37).withTimeout(500),

                new ParallelCommandGroup(

                        new ElevatorPositions(elevator, 34).withTimeout(1500),
                        new MoveIntake(intake, 0).withTimeout(500)),
                new StowAll(arm, elevator, wrist),

                //deja la pieza
                new HighBasket(arm,elevator,wrist),
                new MoveIntake(intake, 0),
                new MoveArm(arm, 90),
                new StowAll(arm, elevator, wrist),

                //se estaciona
                new TurnToAngle(chassis,Rotation2d.fromDegrees(-90)),
                new RamsetteCommand(chassis, Four)



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