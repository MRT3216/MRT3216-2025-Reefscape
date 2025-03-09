package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.BargeCage;
import frc.robot.settings.Constants.BranchSide;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.settings.Constants.ReefBranch;
import frc.robot.settings.FieldPoses;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DriveCommands {

    public static Command driveToBargeClimb(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.defer(
                () -> DriveCommands.driveToPose(FieldPoses.getBargePose(BargeCage.middleCage), drivetrain));
    }

    private static Command driveToPose(Pose2d pose, CommandSwerveDrivetrain drivetrain) {
        return FieldPoses.getDistanceFromRobotPose(pose,
                drivetrain.getRobotPose()) > Constants.PATHING.pathingMinimumDistance
                        ? AutoBuilder.pathfindToPose(pose, Constants.PATHING.pathConstraints, 0)
                        : new DriveToPose(drivetrain, pose);
    }

    public static Command driveToProcessor(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.defer(
                () -> DriveCommands.driveToPose(
                        FieldPoses.getProcessorPose(), drivetrain));
    }

    public static Command driveToLeftCoralStation(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.defer(
                () -> DriveCommands.driveToPose(
                        FieldPoses.getCoralStationPose(CoralStationSide.LEFT), drivetrain));
    }

    public static Command driveToRightCoralStation(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.defer(
                () -> DriveCommands.driveToPose(
                        FieldPoses.getCoralStationPose(CoralStationSide.RIGHT), drivetrain));
    }

    public static Command driveToNearestLeftReefPole(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.defer(() -> driveToNearestReefThenAlign(BranchSide.LEFT, drivetrain));
    }

    public static Command driveToNearestRightReefPole(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.defer(() -> driveToNearestReefThenAlign(BranchSide.RIGHT, drivetrain));
    }

    private static Command driveToNearestReefThenAlign(BranchSide side, CommandSwerveDrivetrain drivetrain) {
        Pose2d reefPose = FieldPoses.getNearestReefFaceInitial(side, drivetrain.getRobotPose());

        return driveToReefPoseThenAlign(reefPose, drivetrain);
    }

    private static Command driveToReefPoseThenAlign(Pose2d reefPose, CommandSwerveDrivetrain drivetrain) {
        Pose2d reefPoseClose = reefPose.transformBy(
                Constants.FIELD_OFFSETS.getReefOffsetPositionClose());

        if (FieldPoses.getDistanceFromRobotPose(reefPose,
                drivetrain.getRobotPose()) < Constants.PATHING.pathingMinimumDistance) {
            return new DriveToPose(drivetrain, reefPoseClose);
        } else {
            return AutoBuilder
                    .pathfindToPose(reefPose, Constants.PATHING.pathConstraints,
                            Constants.PATHING.pathToCloseAlignEndVelocityMPS)
                    .andThen(new DriveToPose(drivetrain, reefPoseClose));
        }
    }

    public static Command driveAndAlignToReefBranch(ReefBranch reefBranch, CommandSwerveDrivetrain drivetrain) {
        return drivetrain.defer(
                () -> DriveCommands.driveToReefPoseThenAlign(
                        FieldPoses.getReefPolePose(reefBranch),
                        drivetrain));
    }

}