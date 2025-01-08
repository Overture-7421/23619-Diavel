package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class GroundMoveIntake extends SequentialCommandGroup {
    private final double distanceFromFloor = 3;
    private final double baseLength = 29.21; // Distance from the arm pivot to the base of the object
    private final double armLength = 50;
    public double targetDistance = 15; // Horizontal distance to the object
    public double adjustment = 0.001;
    private double targetHypotenuse = 0;
    private double requiredAngle = 0;

    public GroundMoveIntake(Arm arm, Elevator elevator, double adjustmentValue) {
        this.adjustment = adjustmentValue;
        this.targetDistance += adjustment;

        // Calculate the required hypotenuse and angle
        targetHypotenuse = Math.sqrt((baseLength * baseLength) + (targetDistance * targetDistance));
        requiredAngle = Math.atan(targetDistance / baseLength);

        // Add sequential commands to move the arm and elevator
        addCommands(
                new MoveArm(arm, requiredAngle),
                new ElevatorPositions(elevator, (targetHypotenuse - armLength - distanceFromFloor))
        );
    }
}
