package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Baskets.HighBasket;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
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
public class HighBasketAndPark extends LinearOpMode {
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
        TrajectoryConfig forwardConfig = new TrajectoryConfig(2, 1);
        forwardConfig.setReversed(false);

        TrajectoryConfig reverseConfig = new TrajectoryConfig(2, 1);
        reverseConfig.setReversed(true);



        Trajectory First = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(0.2,0,Rotation2d.fromDegrees(0))), forwardConfig
        );

        Trajectory Second = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.2,0,Rotation2d.fromDegrees(-35)),
                new Pose2d(1,-0.59,Rotation2d.fromDegrees(-35))), forwardConfig
        );


        Trajectory Third = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1,-0.59,Rotation2d.fromDegrees(0)),
                new Pose2d(0.13,-0.6,Rotation2d.fromDegrees(0))), reverseConfig
        );


        Trajectory Fourth = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1,-0.59,Rotation2d.fromDegrees(-14)),
                new Pose2d(0.85,-0.75,Rotation2d.fromDegrees(-14))), forwardConfig
        );


        /*Trajectory Fifth = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.85,-0.75,Rotation2d.fromDegrees(5)),
                new Pose2d(0.15,-0.82,Rotation2d.fromDegrees(5))), reverseConfig
        );*/


        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new RamsetteCommand(chassis, First),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(-35)),
                new WaitCommand(1000),
                new RamsetteCommand(chassis, Second),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, Third),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(-14)),
                new RamsetteCommand(chassis, Fourth)
                /*new TurnToAngle(chassis, Rotation2d.fromDegrees(5)),
                new RamsetteCommand(chassis, Fifth)*/

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
