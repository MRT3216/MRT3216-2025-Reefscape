package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.ReefBranch;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class AutoCommands {

    public static Command getLeft3PAuto(CommandSwerveDrivetrain drivetrain) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return DriveCommands.driveAndAlignToReefBranch(() -> ReefBranch.J, drivetrain)
                .andThen(DriveCommands.driveToLeftCoralStation(drivetrain))
                .andThen(DriveCommands.driveAndAlignToReefBranch(() -> ReefBranch.K, drivetrain))
                .andThen(DriveCommands.driveToLeftCoralStation(drivetrain))
                .andThen(DriveCommands.driveAndAlignToReefBranch(() -> ReefBranch.L, drivetrain))
                .andThen(DriveCommands.driveToLeftCoralStation(drivetrain));
    }

    public static Command getCenter1PAuto(CommandSwerveDrivetrain drivetrain) {
        return DriveCommands.driveAndAlignToReefBranch(() -> ReefBranch.H, drivetrain);
    }

    public static Command getRight3PAuto(CommandSwerveDrivetrain drivetrain) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return DriveCommands.driveAndAlignToReefBranch(() -> ReefBranch.E, drivetrain)
                .andThen(DriveCommands.driveToRightCoralStation(drivetrain))
                .andThen(DriveCommands.driveAndAlignToReefBranch(() -> ReefBranch.D, drivetrain))
                .andThen(DriveCommands.driveToRightCoralStation(drivetrain))
                .andThen(DriveCommands.driveAndAlignToReefBranch(() -> ReefBranch.C, drivetrain))
                .andThen(DriveCommands.driveToRightCoralStation(drivetrain));
    }
}