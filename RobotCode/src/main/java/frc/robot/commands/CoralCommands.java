package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.subsystems.Coral.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Coral.EndEffector.CoralEndEffectorSubsystem;
import frc.robot.subsystems.Coral.Pivot.CoralPivotSubsystem;

public class CoralCommands {
    public static Command intakeCoralFromStationCommand(
            ElevatorSubsystem elevator,
            CoralPivotSubsystem pivot,
            CoralEndEffectorSubsystem endEffector) {
        // Move the elevator to the coral station height and start the end effector
        // After the coral has been retrieved, move the elevator and pivot to the score prep height
        return moveElevatorAndPivotToHeightCommand(elevator, pivot, () -> POSITIONS.CORAL_STATION)
                .alongWith(endEffector.runEndEffectorCommand().until(endEffector.hasCoral()))
                .andThen(CoralCommands.moveElevatorAndPivotToHeightCommand(
                        elevator, pivot, () -> POSITIONS.SCORE_PREP));
    }

    // Move the elevator and pivot to the specified position
    public static Command moveElevatorAndPivotToHeightCommand(ElevatorSubsystem elevator, CoralPivotSubsystem pivot,
            Supplier<POSITIONS> position) {
        return Commands.defer(() -> (elevator.moveElevatorToHeight(position.get().getHeight())
                .alongWith(pivot.movePivotToAngle(position.get().getAngle()))), Set.of(elevator, pivot));
    }

    public Command scoreCoral(ElevatorSubsystem elevator, CoralPivotSubsystem pivot, CoralEndEffectorSubsystem endEffector){
        return endEffector.outtakeCoralCommand().andThen(moveElevatorAndPivotToHeightCommand(elevator, pivot, ()->POSITIONS.STOW));
    }
}
