package frc.robot.commands;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.settings.Constants.ReefBranch;
import frc.robot.subsystems.Coral.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Coral.Pivot.CoralPivotSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class AutoCommands {

    public static Command getLeft3PAuto(ComboCommands comboCommands) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.J, () -> POSITIONS.L4);
        // .andThen(comboCommands.scoreCoral())
        // .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.LEFT))
        // .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.K, () -> POSITIONS.L4))
        // .andThen(comboCommands.scoreCoral())
        // .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.LEFT))
        // .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.L, () -> POSITIONS.L4))
        // .andThen(comboCommands.scoreCoral())
        // .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.LEFT));
    }

    public static Command getCenter1PAuto(ComboCommands comboCommands) {
        return comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.H, () -> POSITIONS.L4)
                .andThen(comboCommands.scoreCoral());
    }

    public static Command getRight3PAuto(ComboCommands comboCommands) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.E, () -> POSITIONS.L4);
        // .andThen(comboCommands.scoreCoral())
        // .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT))
        // .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.D, () -> POSITIONS.L4))
        // .andThen(comboCommands.scoreCoral())
        // .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT))
        // .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.C, () -> POSITIONS.L4))
        // .andThen(comboCommands.scoreCoral())
        // .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT));
    }

    public static Command getRight1PAuto(ComboCommands comboCommands) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.D, () -> POSITIONS.L3)
                .andThen(comboCommands.scoreCoral())
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT));
    }

    public static Command getRight2PAuto(ComboCommands comboCommands) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.D, () -> POSITIONS.L2)
                .andThen(comboCommands.scoreCoral())
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT))
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.C, () -> POSITIONS.L2))
                .andThen(comboCommands.scoreCoral());
    }

    public static Command driveForward(CommandSwerveDrivetrain drivetrain) {
        SwerveRequest.FieldCentric forwardStraight = new SwerveRequest.FieldCentric();
        return drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)).withTimeout(2);
    }

    public static Command driveForwardL1(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator,
            CoralPivotSubsystem coralPivot, ComboCommands comboCommands) {
        SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();
        return CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
                POSITIONS.L1)
                .andThen(
                        drivetrain.applyRequest(
                                () -> forwardStraight.withVelocityX(0.5)
                                        .withVelocityY(0))
                                .withTimeout(5))
                .andThen(comboCommands.scoreCoralL1());
    }

    public static Command pushForward(CommandSwerveDrivetrain drivetrain) {
        SwerveRequest.FieldCentric forwardStraight = new SwerveRequest.FieldCentric();
        return drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-1.0).withVelocityY(0)).withTimeout(15);
    }
}