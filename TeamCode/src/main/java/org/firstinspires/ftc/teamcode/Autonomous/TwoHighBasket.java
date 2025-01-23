package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RamseteCommand;
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
import org.firstinspires.ftc.teamcode.Commands.GroundGrab;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import java.util.Arrays;


@Autonomous
public class TwoHighBasket extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis = new Chassis(hardwareMap);
        Elevator elevator= new Elevator(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // REVERSE
        TrajectoryConfig backwardConfig = new TrajectoryConfig(0.5, 0.2);
        backwardConfig.setReversed(true);

        // FORWARD
        TrajectoryConfig forwardConfig = new TrajectoryConfig(0.5, 0.2);
        forwardConfig.setReversed(false);

        Trajectory First = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.0,0,Rotation2d.fromDegrees(-45)),
                new Pose2d(0.55,-0.55,Rotation2d.fromDegrees(-45))), forwardConfig
        );

        Trajectory Second  = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.66,-0.66,Rotation2d.fromDegrees(-45)),
                new Pose2d(0.1,-0.1,Rotation2d.fromDegrees(-45))), backwardConfig
        );

        Trajectory Bit = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.55,-0.55,Rotation2d.fromDegrees(-45)),
                new Pose2d(0.66,-0.66,Rotation2d.fromDegrees(-45))), forwardConfig
        );

        Trajectory Third  = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.1,0.1,Rotation2d.fromDegrees(90)),
                new Pose2d(0.1,2,Rotation2d.fromDegrees(90))), forwardConfig
        );

        SequentialCommandGroup FirstCommandGroup = new SequentialCommandGroup(

                new TurnToAngle (chassis, Rotation2d.fromDegrees(-45)),
                new WaitCommand(1000),
                new RamsetteCommand(chassis, First),
                new GroundGrab(arm, elevator),
                new WaitCommand(500),

                new ParallelCommandGroup (
                        new RamsetteCommand(chassis, Bit),
                        new MoveIntake(intake, 1).withTimeout(3000)

                ),

                new MoveIntake(intake,0),
                new StowAll(arm, elevator),
                new RamsetteCommand(chassis, Second),
                new WaitCommand(500),
                new TurnToAngle(chassis,Rotation2d.fromDegrees(90)),
                new RamsetteCommand(chassis, Third)

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
