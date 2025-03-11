package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.BranchSide;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.settings.Constants.ReefBranch;
import frc.robot.settings.FieldPoses;
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

        // TODO: Uncomment these lines when PhotonVision is implemented
        // DriveCommands.readyToPrepElevatorForCoralStation(() -> CoralStationSide.LEFT, drivetrain.getRobotPose())
        //         .onTrue(CoralCommands.intakeCoralFromStationCommand(elevator, coralPivot, coralEndEffector));

        // DriveCommands.readyToPrepElevatorForCoralStation(() -> CoralStationSide.RIGHT, drivetrain.getRobotPose())
        //         .onTrue(CoralCommands.intakeCoralFromStationCommand(elevator, coralPivot, coralEndEffector));

        // #endregion
    }

    private Command driveToReefPoseThenAlignAndScorePrep(Supplier<Pose2d> reefPose, Supplier<POSITIONS> position) {
        Pose2d reefPoseClose = reefPose.get().transformBy(
                Constants.FIELD_OFFSETS.getReefOffsetPositionClose());

        if (FieldPoses.getDistanceFromRobotPose(reefPose.get(),
                drivetrain.getRobotPose()) < Constants.PATHING.pathingMinimumDistance) {

            return new DriveToPose(drivetrain, reefPoseClose)
                    .alongWith(
                            // TODO: Position is current hardcoded for testing; change this back    
                            CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
                                    () -> POSITIONS.L4));
        } else {
            return AutoBuilder
                    .pathfindToPose(reefPose.get(), Constants.PATHING.pathConstraints,
                            Constants.PATHING.pathToCloseAlignEndVelocityMPS)
                    .andThen(new DriveToPose(drivetrain, reefPoseClose)
                            .alongWith(
                                    // TODO: Position is current hardcoded for testing; change this back
                                    CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
                                            () -> POSITIONS.L4)));
        }
    }

    public Command driveToNearestReefThenAlignAndScorePrep(Supplier<BranchSide> side) {
        Supplier<Pose2d> reefPose = () -> FieldPoses.getNearestReefFaceInitial(side.get(), drivetrain.getRobotPose());
        Supplier<POSITIONS> position = elevator.getSelectedPosition();
        System.out.println("Position: " + position.get());
        return drivetrain.defer(() -> this.driveToReefPoseThenAlignAndScorePrep(reefPose, position));
    }

    public Command driveAndAlignToReefBranchAndScorePrep(Supplier<ReefBranch> reefBranch,
            Supplier<POSITIONS> position) {
        return drivetrain.defer(
                () -> this.driveToReefPoseThenAlignAndScorePrep(
                        () -> FieldPoses.getReefPolePose(reefBranch.get()), position));
    }

    public Command retrieveFromCoralStationCommand(Supplier<CoralStationSide> side) {
        return drivetrain.defer(() -> DriveCommands.driveToCoralStation(drivetrain, side));
        // .andThen(CoralCommands.moveElevatorAndPivotToHeightCommand(
        //         elevator, coralPivot, () -> POSITIONS.SCORE_PREP));
    }

    public Command intakeCoralFromStationCommand() {
        // Move the elevator to the coral station height and start the end effector
        // After the coral has been retrieved, move the elevator and pivot to the score prep height
        return Commands.defer(
                () -> CoralCommands
                        .moveElevatorAndPivotToHeightCommand(elevator, coralPivot, () -> POSITIONS.CORAL_STATION)
                        .alongWith(
                                coralEndEffector.runEndEffectorCommand().until(coralEndEffector.hasCoral()))
                        // TODO: TESTING!!!!! Remove timeout
                        .withTimeout(2)
                        .andThen(
                                CoralCommands.moveElevatorAndPivotToHeightCommand(
                                        elevator, coralPivot, () -> POSITIONS.SCORE_PREP)),
                Set.of(elevator, coralPivot, coralEndEffector));
    }

    public Command scoreCoral() {
        // TODO: Adjust timeout
        return coralEndEffector.outtakeCoralCommand().withTimeout(1)
                .andThen(
                        CoralCommands.moveElevatorAndPivotToHeightCommand(
                                elevator, coralPivot, () -> POSITIONS.STOW));
    }
}