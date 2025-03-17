package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.BargeCage;
import frc.robot.settings.Constants.CoralStationSide;
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

    public static Command driveToCoralStation(CommandSwerveDrivetrain drivetrain, Supplier<CoralStationSide> side) {
        return drivetrain.defer(
                () -> DriveCommands.driveToPose(
                        FieldPoses.getCoralStationPose(side.get()),
                        drivetrain));
    }

    // TODO: Uncomment these lines when PhotonVision is implemented
    // public static Trigger readyToPrepElevatorForCoralStation(Supplier<CoralStationSide> side,
    //         Supplier<Pose2d> robotPoseSupplier) {
    //     return new Trigger(
    //             () -> (FieldPoses.getDistanceFromRobotPose(FieldPoses.getCoralStationPose(side.get()),
    //                     robotPoseSupplier) < FIELD_OFFSETS.elevatorPrepCoralStationDistance.in(Meters)));
    // }
}