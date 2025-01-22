package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;

import java.util.Arrays;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

/* This autonomous cycles between the pre-established alliance-specific specimens placed on the
* field and the observation zone for the human player to attach clips*/

@Autonomous
public class ThreeSpecimensToObservationZone extends LinearOpMode {
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


        TrajectoryConfig forwardConfig = new TrajectoryConfig(0.5, 0.2);
        forwardConfig.setReversed(false);
        Trajectory FirstSample = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0,0,Rotation2d.fromDegrees(0 )),
                new Pose2d(1.45,-0.85 ,Rotation2d.fromDegrees(0))), forwardConfig
        );

        TrajectoryConfig forwardConfig2 = new TrajectoryConfig(0.5, 0.2);
        forwardConfig.setReversed(false);
        Trajectory GoToObservationZone1 = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.45,-0.85,Rotation2d.fromDegrees(0)),
                new Pose2d(0,-1 ,Rotation2d.fromDegrees(0))), forwardConfig2
        );

        TrajectoryConfig forwardConfig3 = new TrajectoryConfig(0.5, 0.2);
        forwardConfig.setReversed(false);
        Trajectory SecondSample = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0,-1,Rotation2d.fromDegrees(0)),
                new Pose2d(1.45,-1.07 ,Rotation2d.fromDegrees(0))), forwardConfig3
        );

        TrajectoryConfig forwardConfig4 = new TrajectoryConfig(0.5, 0.2);
        forwardConfig.setReversed(false);
        Trajectory GoToObservationZone2 = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.45,-1.07,Rotation2d.fromDegrees(0)),
                new Pose2d(0,-1 ,Rotation2d.fromDegrees(0))), forwardConfig4
        );

        TrajectoryConfig forwardConfig5 = new TrajectoryConfig(0.5, 0.2);
        forwardConfig.setReversed(false);
        Trajectory ThirdSample = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0,-1 ,Rotation2d.fromDegrees(-60 /*Pending Test*/ )),
                new Pose2d(1.45,-1.30 ,Rotation2d.fromDegrees(-60 /*Pending Test*/ ))), forwardConfig5
        );

        TrajectoryConfig forwardConfig6 = new TrajectoryConfig(0.5, 0.2);
        forwardConfig.setReversed(false);
        Trajectory GoToObservationZone3 = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.40,-1.30,Rotation2d.fromDegrees(0)),
                new Pose2d(0,-1 ,Rotation2d.fromDegrees(0))), forwardConfig6
        );

        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, FirstSample),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, GoToObservationZone1),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, SecondSample),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, GoToObservationZone2),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, ThirdSample),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis,GoToObservationZone3)

        );

        waitForStart();
        chassis.reset(FirstSample.getInitialPose());
        CommandScheduler.getInstance().schedule(testCommandGroup);
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
