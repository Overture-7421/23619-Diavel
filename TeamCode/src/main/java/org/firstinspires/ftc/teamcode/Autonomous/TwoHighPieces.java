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
public class TwoHighPieces extends LinearOpMode {
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
        Trajectory UP = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(0.8,0,Rotation2d.fromDegrees(0))), forwardConfig
        );

        TrajectoryConfig backwardConfig = new TrajectoryConfig(0.5, 0.2);
        backwardConfig.setReversed(true);
        Trajectory DOWN = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.8,0,Rotation2d.fromDegrees(0)),
                new Pose2d(0.4,0,Rotation2d.fromDegrees(0))), backwardConfig
        );


        TrajectoryConfig rotationLeftConfig = new TrajectoryConfig(0.5, 0.2);
        rotationLeftConfig.setReversed(true);
        Trajectory Left = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.4,0,Rotation2d.fromDegrees(90)),
                new Pose2d(0.4,1.2,Rotation2d.fromDegrees(90))), rotationLeftConfig
        );

        TrajectoryConfig rotationCenterConfig = new TrajectoryConfig(0.5, 0.2);
        rotationCenterConfig.setReversed(true);
        Trajectory Piece = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.4,1.2,Rotation2d.fromDegrees(0)),
                new Pose2d(0.7,1.2,Rotation2d.fromDegrees(0))), rotationCenterConfig
        );

        TrajectoryConfig backwardConfig2 = new TrajectoryConfig(0.5, 0.2);
        backwardConfig2.setReversed(false);
        Trajectory Basket = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.7,1.2,Rotation2d.fromDegrees(0)),
                new Pose2d(0.2,1.2,Rotation2d.fromDegrees(0))), backwardConfig2
        );

        TrajectoryConfig rotationRightConfig = new TrajectoryConfig(0.5, 0.2);
        rotationRightConfig.setReversed(false);
        Trajectory Accomodation = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.2,1.2,Rotation2d.fromDegrees(-45)),
                new Pose2d(0.1,1.2,Rotation2d.fromDegrees(-45))), rotationRightConfig
        );

        TrajectoryConfig forwardConfig2 = new TrajectoryConfig(0.5, 0.2);
        forwardConfig2.setReversed(true);
        Trajectory Littlebit = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.1,1.2,Rotation2d.fromDegrees(-45)),
                new Pose2d(0.2,1.2,Rotation2d.fromDegrees(-45))), forwardConfig2
        );

        TrajectoryConfig backwardConfig3 = new TrajectoryConfig(0.5, 0.2);
        backwardConfig3.setReversed(false);
        Trajectory Littlebitbacck = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.2,1.2,Rotation2d.fromDegrees(-45)),
                new Pose2d(0.0,1.2,Rotation2d.fromDegrees(-45))), backwardConfig3
        );


        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new RamsetteCommand(chassis, UP),
                new WaitCommand(1000),
                new RamsetteCommand(chassis, DOWN),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(90)),
                new RamsetteCommand(chassis, Left),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new RamsetteCommand(chassis, Piece),
                new WaitCommand(1000),
                new RamsetteCommand(chassis, Basket),
                new TurnToAngle (chassis, Rotation2d.fromDegrees(-45)),
                new RamsetteCommand(chassis, Accomodation),
                new WaitCommand(1000),
                new RamsetteCommand(chassis, Littlebit),
                new WaitCommand(1000),
                new RamsetteCommand(chassis, Littlebitbacck)
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

