package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.ALGAE.PIVOT.Positions;
import frc.robot.subsystems.Algae.Pivot.AlgaePivotSubsystem;
import frc.robot.subsystems.Algae.Rollers.AlgaeRollersSubsystem;

public class AlgaeCommands {
    public static Command intakeAlgae(AlgaePivotSubsystem pivot, AlgaeRollersSubsystem rollers) {
        return pivot.movePivotToAngle(Positions.INTAKING.getAngle())
                .alongWith(rollers.intakeAlgae())
                .until(rollers.hasAlgaeTrigger())
                .andThen(pivot.movePivotToAngle((Positions.STOW_SCORING.getAngle())));
    }

    public static Command scoreAlgae(AlgaePivotSubsystem pivot, AlgaeRollersSubsystem rollers) {
        return rollers.scoreAlgae();
    }
}
