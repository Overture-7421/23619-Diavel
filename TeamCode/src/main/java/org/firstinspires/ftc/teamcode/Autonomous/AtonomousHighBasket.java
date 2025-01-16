package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Baskets.HighBasket;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RamseteCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.RamsetteCommand;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.TurnToAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.


import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Autonomous
public class AtonomousHighBasket extends LinearOpMode{
    Chassis chassis;
    Intake intake;
    Arm arm;
    Elevator elevator;

    @Override

    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();


        chassis = new Chassis(hardwareMap);
        intake = new Intake(hardwareMap);
        arm = new Arm(hardwareMap);
        elevator = new Elevator(hardwareMap);


        TrajectoryConfig FORWARDConfig = new TrajectoryConfig(0.5, 0.2);
        FORWARDConfig.setReversed(false);
        Trajectory FORWARD = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(0.55,0.55,Rotation2d.fromDegrees(0))), FORWARDConfig
        );

        SequentialCommandGroup testCommandGroupFirst = new SequentialCommandGroup(
             new TurnToAngle(chassis, Rotation2d.fromDegrees(-45)),
                new RamsetteCommand(chassis, FORWARD),
                new GroundGrab(arm, elevator)



              /*  new RamsetteCommand(chassis, UP),
                new HighBasket(arm, elevator),
                new WaitCommand(1500),
                new StowAll(arm, elevator),
                new WaitCommand(1000),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(-90)),
                new RamsetteCommand(chassis, RIGHT),
                new GroundGrab(arm, elevator),
                new WaitCommand(1000),
                new StowAll(arm, elevator),
                new WaitCommand(1500),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(180)),
                new RamsetteCommand(chassis, getToSecondPosition),
                new WaitCommand(500),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(90)),
                new RamsetteCommand(chassis, getToStartingPosition)*/

        );
        ParallelCommandGroup takeFromGround = new ParallelCommandGroup(
                new MoveIntake(intake, 1),
                new StowAll(arm, elevator)
        );

        TrajectoryConfig backwardConfig = new TrajectoryConfig(0.5, 0.2);
        backwardConfig.setReversed(true);
        Trajectory BACKWARD = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.55,0.55,Rotation2d.fromDegrees(0)),
                new Pose2d(0,0,Rotation2d.fromDegrees(0))), backwardConfig
        );

        SequentialCommandGroup testCommandGroupSecond= new SequentialCommandGroup(
                new RamsetteCommand(chassis, BACKWARD)

                );

        waitForStart();
        chassis.reset(FORWARD.getInitialPose());
        CommandScheduler.getInstance().schedule(testCommandGroupFirst);
        while (opModeIsActive ()){
            CommandScheduler.getInstance().run();

            Pose2d pose = chassis.getPose();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getRotation().getDegrees());
            telemetry.update();


        }

    }

}
