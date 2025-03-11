package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.settings.Constants.ReefBranch;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class AutoCommands {

    public static Command getLeft3PAuto(ComboCommands comboCommands) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.J, () -> POSITIONS.L4)
                .andThen(comboCommands.scoreCoral())
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.LEFT))
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.K, () -> POSITIONS.L4))
                .andThen(comboCommands.scoreCoral())
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.LEFT))
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.L, () -> POSITIONS.L4))
                .andThen(comboCommands.scoreCoral())
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.LEFT));
    }

    public static Command getCenter1PAuto(ComboCommands comboCommands) {
        return comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.H, () -> POSITIONS.L4)
                .andThen(comboCommands.scoreCoral());
    }

    public static Command getRight3PAuto(ComboCommands comboCommands) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.E, () -> POSITIONS.L4)
                .andThen(comboCommands.scoreCoral())
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT))
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.D, () -> POSITIONS.L4))
                .andThen(comboCommands.scoreCoral())
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT))
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.C, () -> POSITIONS.L4))
                .andThen(comboCommands.scoreCoral())
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT));
    }

    public static Command driveForward(CommandSwerveDrivetrain drivetrain) {
        SwerveRequest.FieldCentric forwardStraight = new SwerveRequest.FieldCentric();
        return drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)).withTimeout(2);
    }
}