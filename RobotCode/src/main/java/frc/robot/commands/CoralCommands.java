package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.subsystems.Coral.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Coral.Pivot.CoralPivotSubsystem;

public class CoralCommands {
    // Move the elevator and pivot to the specified position, but delay moving the pivot until
    // the elevator reaches its goal
    public static Command moveElevatorAndPivotToHeightCommandDelayPivot(
            ElevatorSubsystem elevator,
            CoralPivotSubsystem pivot,
            Supplier<POSITIONS> position) {
        return Commands.defer(
                () -> (pivot.movePivotToAngle(POSITIONS.SCORE_PREP.getAngle())
                        .alongWith(elevator.moveElevatorToHeight(position.get().getHeight()))
                        .andThen(pivot.movePivotToAngle(position.get().getAngle()))),
                Set.of(elevator, pivot));
    }

    // Move the elevator and pivot to the specified position and immediately
    // move the pivot
    public static Command moveElevatorAndPivotToHeightCommand(
            ElevatorSubsystem elevator,
            CoralPivotSubsystem pivot,
            Supplier<POSITIONS> position) {
        return Commands.defer(
                () -> (elevator.moveElevatorToHeight(position.get().getHeight())
                        .alongWith(pivot.movePivotToAngle(position.get().getAngle()))),
                Set.of(elevator, pivot));
    }
}
