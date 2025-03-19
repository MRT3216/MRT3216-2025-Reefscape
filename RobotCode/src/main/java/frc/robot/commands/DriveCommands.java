package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.settings.Constants.FIELD_OFFSETS;
import frc.robot.settings.Constants.PATHING;
import frc.robot.settings.FieldPoses;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DriveCommands {
    // public static Command driveToBargeClimb(CommandSwerveDrivetrain drivetrain) {
    //     return DriveCommands.driveToPose(FieldPoses.getBargePose(BargeCage.middleCage), drivetrain);
    // }

    private static Command driveToPose(Supplier<Pose2d> pose, CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> FieldPoses.getDistanceFromRobotPose(pose.get(),
                drivetrain.getRobotPose().get()) > Constants.PATHING.pathingMinimumDistance
                        ? AutoBuilder.pathfindToPose(pose.get(), Constants.PATHING.pathConstraints,
                                PATHING.pathToCloseAlignEndVelocityMPS)
                        : new CloseDriveToPose(drivetrain, pose.get(), false),
                Set.of(drivetrain));
    }

    public static Command driveToProcessor(CommandSwerveDrivetrain drivetrain) {
        return DriveCommands.driveToPose(
                () -> FieldPoses.getProcessorPose(), drivetrain);
    }

    public static Command driveToCoralStation(CommandSwerveDrivetrain drivetrain, Supplier<CoralStationSide> side) {
        return DriveCommands.driveToPose(
                () -> FieldPoses.getCoralStationPose(side.get()),
                drivetrain);
    }

    public static Trigger readyToPrepElevatorForCoralStation(Supplier<CoralStationSide> side,
            Supplier<Pose2d> robotPoseSupplier) {
        return new Trigger(
                () -> FieldPoses.getDistanceFromRobotPose(FieldPoses.getCoralStationPose(side.get()),
                        robotPoseSupplier.get()) < FIELD_OFFSETS.elevatorPrepCoralStationDistance.in(Meters));
    }

    // public static Trigger shouldStowElevator(Supplier<CoralStationSide> side,
    //         Supplier<Pose2d> robotPoseSupplier) {
    //     return new Trigger(
    //             () -> (FieldPoses.getDistanceFromRobotPose(FieldPoses.getCoralStationPose(side.get()),
    //                     robotPoseSupplier.get()) > FIELD_OFFSETS.elevatorPrepCoralStationDistance.in(Meters)));
    // }
}