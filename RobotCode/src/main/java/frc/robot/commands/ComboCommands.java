package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.subsystems.Coral.EndEffector.CoralEndEffectorSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class ComboCommands {
    public static Command getCoralCommand(CoralStationSide side,
            CommandSwerveDrivetrain drivetrain,
            CoralEndEffectorSubsystem endEffector) {
        return drivetrain.defer(() -> Commands.parallel(
                side == CoralStationSide.LEFT ? DriveCommands.driveToLeftCoralStation(drivetrain)
                        : DriveCommands.driveToRightCoralStation(drivetrain),
                endEffector.intakeCoral()));
    }
}