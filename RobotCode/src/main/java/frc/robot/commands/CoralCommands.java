package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.subsystems.Coral.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Coral.Pivot.CoralPivotSubsystem;

public class CoralCommands {
    // Move the elevator and pivot to the specified position, but delay moving the pivot until
    // the elevator reaches its goal
    public static Command moveElevatorAndPivotToHeightCommandDelayPivot(
            ElevatorSubsystem elevator,
            CoralPivotSubsystem pivot,
            POSITIONS position) {
        return pivot.movePivotToAngle(POSITIONS.SCORE_PREP)
                .alongWith(elevator.moveToTargetPosition())
                .andThen(pivot.movePivotToAngle(position));
    }

    // Move the elevator and pivot to the specified position and immediately
    // move the pivot
    public static Command moveElevatorAndPivotToHeightCommand(
            ElevatorSubsystem elevator,
            CoralPivotSubsystem pivot,
            POSITIONS position) {
        return elevator.moveElevatorToHeight(position)
                .alongWith(pivot.movePivotToAngle(position));
    }
}
