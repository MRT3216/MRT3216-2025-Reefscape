package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.BranchSide;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.settings.Constants.ReefBranch;
import frc.robot.settings.FieldPoses;
import frc.robot.subsystems.Coral.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Coral.EndEffector.CoralEndEffectorSubsystem;
import frc.robot.subsystems.Coral.Pivot.CoralPivotSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class ComboCommands {
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorSubsystem elevator;
    private final CoralPivotSubsystem coralPivot;
    private final CoralEndEffectorSubsystem coralEndEffector;

    public ComboCommands(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator, CoralPivotSubsystem coralPivot,
            CoralEndEffectorSubsystem coralEndEffector) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.coralPivot = coralPivot;
        this.coralEndEffector = coralEndEffector;

        this.configureBindings();
    }

    public void configureBindings() {
        // #region Triggers

        // #endregion
    }

    public Command driveToReefPoseThenAlignAndScorePrep(Supplier<Pose2d> reefPose, Supplier<POSITIONS> position) {
        Pose2d reefPoseClose = reefPose.get().transformBy(
                Constants.FIELD_OFFSETS.getReefOffsetPositionClose());

        if (FieldPoses.getDistanceFromRobotPose(reefPose.get(),
                drivetrain.getRobotPose().get()) < Constants.PATHING.pathingMinimumDistance) {
            return new CloseDriveToPose(drivetrain, reefPoseClose, true)
                    .alongWith(
                            CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
                                    position.get()));
        } else {
            return AutoBuilder.pathfindToPose(reefPose.get(), Constants.PATHING.pathConstraints,
                    Constants.PATHING.pathToCloseAlignEndVelocityReefMPS)
                    .andThen(new CloseDriveToPose(drivetrain, reefPoseClose, true)
                            .alongWith(
                                    CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
                                            position.get())));
        }
    }

    public Command driveToNearestReefThenAlignAndScorePrep(Supplier<POSITIONS> position, Supplier<BranchSide> side) {
        return drivetrain.defer(() -> this.driveToReefPoseThenAlignAndScorePrep(
                () -> FieldPoses.getNearestReefFaceInitial(side.get(), drivetrain.getRobotPose().get()),
                () -> position.get()));
    }

    public Command driveAndAlignToReefBranchAndScorePrep(Supplier<ReefBranch> reefBranch,
            Supplier<POSITIONS> position) {
        return drivetrain.defer(() -> this.driveToReefPoseThenAlignAndScorePrep(
                () -> FieldPoses.getReefPolePose(reefBranch.get()), position));
    }

    public Command retrieveFromCoralStationCommand(Supplier<CoralStationSide> side) {
        return DriveCommands.driveToCoralStation(drivetrain, side)
                .alongWith(this.intakeCoralFromStationCommand());
    }

    /**
    Moves the elevator to the coral station height and starts the end effector until
    coral is intaked, then moves the elevator and pivot to the score prep height.
    */
    public Command intakeCoralFromStationCommand() {
        return CoralCommands
                // Move the elevator to the coral station height
                .moveElevatorAndPivotToHeightCommand(elevator, coralPivot, POSITIONS.CORAL_STATION)
                .alongWith(
                        // Start the end effector until coral is intaked
                        coralEndEffector.intakeCoralCommand().until(coralEndEffector.hasCoral()))
                .andThen(
                        // Move the elevator and pivot to the score prep height
                        CoralCommands.moveElevatorAndPivotToHeightCommand(
                                elevator, coralPivot, POSITIONS.SCORE_PREP));
    }

    /*
     * Score the coral and then move the elevator and pivot to the stow position
     * @return The command to score the coral
     */
    public Command scoreCoral() {
        return coralEndEffector.outtakeCoralCommand().withTimeout(1)
                .andThen(
                        CoralCommands.moveElevatorAndPivotToHeightCommand(
                                elevator, coralPivot, POSITIONS.STOW));
    }

    /*
     * Score the coral at level 1 and then move the elevator and pivot to the stow position
     * @return The command to score the coral at level 1
     */
    public Command scoreCoralL1() {
        return coralEndEffector.outtakeCoralCommandL1().withTimeout(1)
                .andThen(
                        CoralCommands.moveElevatorAndPivotToHeightCommand(
                                elevator, coralPivot, POSITIONS.STOW));
    }

    /*
     * Move the elevator and pivot to the score prep height
     * @return The command to move the elevator and pivot to the score prep height
     */
    public Command scorePrepElevator() {
        return CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot, POSITIONS.SCORE_PREP);
    }
}