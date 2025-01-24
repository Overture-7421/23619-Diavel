package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.TurnToAngle;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.RamsetteCommand;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

/* This autonomous first scores a high chamber specimen, the travels to the neutral samples, scores
 one on the high basket and then parks.*/

@Autonomous
public class SamplesToObservationZone extends LinearOpMode {
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

        // TRAJECTORY CONFIGS
        TrajectoryConfig forwardConfig = new TrajectoryConfig(1, 0.5);
        forwardConfig.setReversed(false);

        TrajectoryConfig reverseConfig = new TrajectoryConfig(1, 0.5);
        reverseConfig.setReversed(true);



        Trajectory First = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(0.19,0,Rotation2d.fromDegrees(0))), forwardConfig
        );

        Trajectory Second = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.19,0,Rotation2d.fromDegrees(-25)),
                new Pose2d(1.36,-0.53,Rotation2d.fromDegrees(-25))), forwardConfig
        );


        Trajectory Third = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.36,-0.53,Rotation2d.fromDegrees(0)),
                new Pose2d(0.25,-0.6,Rotation2d.fromDegrees(0))), reverseConfig
        );


        Trajectory Fourth = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.15,-0.6,Rotation2d.fromDegrees(0)),
                new Pose2d(1.5,-0.6,Rotation2d.fromDegrees(0))), forwardConfig
        );


        Trajectory Fifth = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.5,-0.6,Rotation2d.fromDegrees(-45)),
                new Pose2d(1.7,-0.8,Rotation2d.fromDegrees(-45))), reverseConfig
        );

        Trajectory Sixth = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.5,-0.8,Rotation2d.fromDegrees(0)),
                new Pose2d(0.25,-0.8,Rotation2d.fromDegrees(0))), forwardConfig
        );

        Trajectory Seventh = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.25,-0.8,Rotation2d.fromDegrees(0)),
                new Pose2d(1.46,-0.8,Rotation2d.fromDegrees(0))), reverseConfig
        );


        Trajectory Eighth = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.46,-0.8,Rotation2d.fromDegrees(-90)),
                new Pose2d(1.46,-1,Rotation2d.fromDegrees(-90))), reverseConfig
        );


        Trajectory Ninth = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.46,-1,Rotation2d.fromDegrees(0)),
                new Pose2d(0.25,-1,Rotation2d.fromDegrees(0))), reverseConfig
        );



        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new RamsetteCommand(chassis, First),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(-25)),
                new WaitCommand(500),
                new RamsetteCommand(chassis, Second),
                new WaitCommand(500),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, Third),
                new WaitCommand(500),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, Fourth),
                new WaitCommand(500),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(-45)),
                new RamsetteCommand(chassis, Fifth)/*,
                new WaitCommand(500),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, Sixth),
                new WaitCommand(500),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, Seventh),
                new WaitCommand(500),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(-90)),
                new RamsetteCommand(chassis, Eighth),
                new WaitCommand(500),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, Ninth)
                */

        );

        waitForStart();
        chassis.reset(First.getInitialPose());
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
