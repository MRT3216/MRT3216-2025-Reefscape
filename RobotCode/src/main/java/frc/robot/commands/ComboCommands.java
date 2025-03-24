package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

        // Robot is near the left coral station, so move elevator and start end effector
        // DriveCommands.readyToPrepElevatorForCoralStation(() -> CoralStationSide.LEFT, robotPoseSupplier)
        //         .onTrue(this.intakeCoralFromStationCommand());

        //Robot is near the right coaral station, so move elevator and start end effector
        // DriveCommands.readyToPrepElevatorForCoralStation(() -> CoralStationSide.RIGHT, robotPoseSupplier)
        //         .onTrue(this.intakeCoralFromStationCommand());

        // Robot is moving away from the left coral station, 
        // so stow the elevator if the end effector has no coral
        // DriveCommands.shouldStowElevator(() -> CoralStationSide.LEFT, drivetrain.getRobotPose())
        //         .onTrue(CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
        //                 POSITIONS.STOW).onlyIf(coralEndEffector.noCoral()));

        // // Robot is moving away from the right coral station, 
        // // so stow the elevator if the end effector has no coral
        // DriveCommands.shouldStowElevator(() -> CoralStationSide.RIGHT, drivetrain.getRobotPose())
        //         .onTrue(CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
        //                 POSITIONS.STOW).onlyIf(coralEndEffector.noCoral()));

        // public Trigger readyToScoreCoral() {
        //     return //coralEndEffector.hasCoral().and(
        //     elevator.atGoal().and(coralPivot.atGoal());
        // }

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

    // public Command retrieveFromCoralStationCommandAuto(Supplier<CoralStationSide> side) {
    //     return DriveCommands.driveToCoralStation(drivetrain, side)
    //             .alongWith( // Move the elevator to the coral station height
    //                     CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
    //                             POSITIONS.CORAL_STATION))
    //             .alongWith(
    //                     // Start the end effector until coral is intaked
    //                     coralEndEffector.runEndEffectorCommand())
    //             .until(coralEndEffector.hasCoral())
    //             .andThen(// Move the elevator and pivot to the score prep height
    //                     CoralCommands.moveElevatorAndPivotToHeightCommand(
    //                             elevator, coralPivot, POSITIONS.SCORE_PREP));
    // }

    // Moves the elevator to the coral station height and starts the end effector until
    // coral is intaked, then moves the elevator and pivot to the score prep height
    public Command intakeCoralFromStationCommand() {
        return CoralCommands
                // Move the elevator to the coral station height
                .moveElevatorAndPivotToHeightCommand(elevator, coralPivot, POSITIONS.CORAL_STATION)
                .alongWith(
                        // Start the end effector until coral is intaked
                        coralEndEffector.runEndEffectorCommand().until(coralEndEffector.hasCoral()))
                // TODO: Remove for real robot
                .withTimeout(4)
                .andThen(
                        // Move the elevator and pivot to the score prep height
                        CoralCommands.moveElevatorAndPivotToHeightCommand(
                                elevator, coralPivot, POSITIONS.SCORE_PREP));
    }

    public Command scoreCoral() {
        return coralEndEffector.outtakeCoralCommand().withTimeout(1)
                .andThen(
                        CoralCommands.moveElevatorAndPivotToHeightCommand(
                                elevator, coralPivot, POSITIONS.STOW));
    }

    public Command scoreCoralL1() {
        return coralEndEffector.outtakeCoralCommandL1().withTimeout(1)
                .andThen(
                        CoralCommands.moveElevatorAndPivotToHeightCommand(
                                elevator, coralPivot, POSITIONS.STOW));
    }
}