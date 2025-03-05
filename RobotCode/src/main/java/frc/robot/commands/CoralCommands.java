package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.subsystems.Coral.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Coral.Pivot.CoralPivotSubsystem;

public class CoralCommands {
    public static Command moveElevatorAndPivotToHeight(ElevatorSubsystem elevator, CoralPivotSubsystem pivot,
            POSITIONS position) {
        return elevator.moveElevatorToHeight(position.getHeight())
                .alongWith(pivot.movePivotToAngle(position.getAngle()));
    }
}
