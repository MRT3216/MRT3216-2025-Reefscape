package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.settings.Constants.ReefBranch;

public class AutoCommands {
    public static Command getCenter1PAuto(ComboCommands comboCommands) {
        return comboCommands.scorePrepElevator()
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.H, () -> POSITIONS.L4))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(comboCommands.scoreCoral());
    }

    public static Command getLeft3PAuto(ComboCommands comboCommands) {
        return comboCommands.scorePrepElevator()
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.J, () -> POSITIONS.L4))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(comboCommands.scoreCoral())
                .andThen(Commands.waitSeconds(0.5))
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.LEFT))
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.K, () -> POSITIONS.L4))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(comboCommands.scoreCoral())
                .andThen(Commands.waitSeconds(0.5))
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.LEFT))
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.L, () -> POSITIONS.L4))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(comboCommands.scoreCoral());
    }

    public static Command getRight3PAuto(ComboCommands comboCommands) {
        return comboCommands.scorePrepElevator()
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.E, () -> POSITIONS.L4))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(comboCommands.scoreCoral())
                .andThen(Commands.waitSeconds(0.5))
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT))
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.D, () -> POSITIONS.L4))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(comboCommands.scoreCoral())
                .andThen(Commands.waitSeconds(0.5))
                .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT))
                .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.C, () -> POSITIONS.L4))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(comboCommands.scoreCoral());
    }

    // public static Command getLeft1PAuto(ComboCommands comboCommands) {
    //     return comboCommands.scorePrepElevator()
    //             .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.J, () -> POSITIONS.L4))
    //             .andThen(Commands.waitSeconds(0.5))
    //             .andThen(comboCommands.scoreCoral())
    //             .andThen(Commands.waitSeconds(1))
    //             .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.LEFT));
    // }

    // // TODO: Fix this for competition field
    // public static Command getRight1PAuto(ComboCommands comboCommands) {
    //     return comboCommands.scorePrepElevator()
    //             .andThen(comboCommands.driveAndAlignToReefBranchAndScorePrep(() -> ReefBranch.D, () -> POSITIONS.L4))
    //             .andThen(Commands.waitSeconds(0.5))
    //             .andThen(comboCommands.scoreCoral())
    //             .andThen(Commands.waitSeconds(1))
    //             .andThen(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT));
    // }

    // public static Command driveForward(CommandSwerveDrivetrain drivetrain) {
    //     SwerveRequest.FieldCentric forwardStraight = new SwerveRequest.FieldCentric();
    //     return drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)).withTimeout(2);
    // }

    // public static Command driveForwardL1(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator,
    //         CoralPivotSubsystem coralPivot, ComboCommands comboCommands) {
    //     SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();
    //     return CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
    //             POSITIONS.L1)
    //             .andThen(
    //                     drivetrain.applyRequest(
    //                             () -> forwardStraight.withVelocityX(0.5)
    //                                     .withVelocityY(0))
    //                             .withTimeout(5))
    //             .andThen(comboCommands.scoreCoralL1());
    // }

    // public static Command pushForward(CommandSwerveDrivetrain drivetrain) {
    //     SwerveRequest.FieldCentric forwardStraight = new SwerveRequest.FieldCentric();
    //     return drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-1.0).withVelocityY(0)).withTimeout(15);
    // }
}