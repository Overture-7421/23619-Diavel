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


        TrajectoryConfig forwardConfig = new TrajectoryConfig(0.5, 0.2);
        forwardConfig.setReversed(false);
        Trajectory UP = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(2,0,Rotation2d.fromDegrees(0))), forwardConfig
        );

        TrajectoryConfig backwardConfig = new TrajectoryConfig(0.5, 0.2);
        backwardConfig.setReversed(false);
        Trajectory RIGHT = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(2,0,Rotation2d.fromDegrees(-90)),
                new Pose2d(2,-2,Rotation2d.fromDegrees(-90))), backwardConfig
        );

        TrajectoryConfig forwardConfig2 = new TrajectoryConfig(0.5, 0.2);
        backwardConfig.setReversed(true);
        Trajectory getToSecondPosition = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(2,-2,Rotation2d.fromDegrees(180)),
                new Pose2d(2,0,Rotation2d.fromDegrees(180))), forwardConfig2
        );

        TrajectoryConfig forwardConfig3 = new TrajectoryConfig(0.5, 0.2);
        backwardConfig.setReversed(false);
        Trajectory getToStartingPosition = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(2,0,Rotation2d.fromDegrees(90)),
                new Pose2d(0,0,Rotation2d.fromDegrees(90))), forwardConfig3
        );


        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new RamsetteCommand(chassis, UP),
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
                new RamsetteCommand(chassis, getToStartingPosition)

        );

        waitForStart();
        chassis.reset(UP.getInitialPose());
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
