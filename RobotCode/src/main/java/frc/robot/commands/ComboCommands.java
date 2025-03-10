package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.BranchSide;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.subsystems.Algae.Pivot.AlgaePivotSubsystem;
import frc.robot.subsystems.Algae.Rollers.AlgaeRollersSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Coral.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Coral.EndEffector.CoralEndEffectorSubsystem;
import frc.robot.subsystems.Coral.Pivot.CoralPivotSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class ComboCommands {
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorSubsystem elevator;
    private final CoralPivotSubsystem coralPivot;
    private final CoralEndEffectorSubsystem coralEndEffector;
    private final AlgaePivotSubsystem algaePivot;
    private final AlgaeRollersSubsystem algaeRollers;
    private final ClimberSubsystem climber;

    public ComboCommands(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator, CoralPivotSubsystem coralPivot,
            CoralEndEffectorSubsystem coralEndEffector, AlgaePivotSubsystem algaePivot,
            AlgaeRollersSubsystem algaeRollers, ClimberSubsystem climber) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.coralPivot = coralPivot;
        this.coralEndEffector = coralEndEffector;
        this.algaePivot = algaePivot;
        this.algaeRollers = algaeRollers;
        this.climber = climber;

        this.configureBindings();
    }

    public void configureBindings() {
        // #region Triggers

        DriveCommands.readyToPrepElevatorForCoralStation(() -> CoralStationSide.LEFT, drivetrain.getRobotPose())
                .onTrue(CoralCommands.intakeCoralFromStationCommand(elevator, coralPivot, coralEndEffector));

        // #endregion
    }

    public Command retrieveFromCoralStationCommand(Supplier<CoralStationSide> side) {
        return DriveCommands.driveToLeftCoralStation(drivetrain);
                // .andThen(CoralCommands.moveElevatorAndPivotToHeightCommand(
                //         elevator, coralPivot, () -> POSITIONS.SCORE_PREP));
    }

    public Command scoreCoralOnReef(Supplier<BranchSide> side) {
        if (side.get() == BranchSide.LEFT)
            return DriveCommands.driveToNearestReefThenAlign(side, drivetrain);
        else
            return  DriveCommands.driveToNearestReefThenAlign(side, drivetrain);
    }
}