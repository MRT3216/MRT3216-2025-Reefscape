package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.settings.Constants.FIELD_OFFSETS;
import frc.robot.settings.FieldPoses;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DriveCommands {
    // public static Command driveToBargeClimb(CommandSwerveDrivetrain drivetrain) {
    //     return DriveCommands.driveToPose(FieldPoses.getBargePose(BargeCage.middleCage), drivetrain);
    // }

    private static Command driveToPose(Supplier<Pose2d> pose, CommandSwerveDrivetrain drivetrain) {
        return FieldPoses.getDistanceFromRobotPose(pose,
                drivetrain.getRobotPose()).get() > Constants.PATHING.pathingMinimumDistance
                        ? AutoBuilder.pathfindToPose(pose.get(), Constants.PATHING.pathConstraints, 0)
                        : new DriveToPose(drivetrain, pose.get());
    }

    public static Command driveToProcessor(CommandSwerveDrivetrain drivetrain) {
        return DriveCommands.driveToPose(
                FieldPoses.getProcessorPose(), drivetrain);
    }

    public static Command driveToCoralStation(CommandSwerveDrivetrain drivetrain, Supplier<CoralStationSide> side) {
        return DriveCommands.driveToPose(
                FieldPoses.getCoralStationPose(side),
                drivetrain);
    }

    public static Trigger readyToPrepElevatorForCoralStation(Supplier<CoralStationSide> side,
            Supplier<Pose2d> robotPoseSupplier) {
        return new Trigger(
                () -> (FieldPoses.getDistanceFromRobotPose(FieldPoses.getCoralStationPose(side),
                        robotPoseSupplier).get() < FIELD_OFFSETS.elevatorPrepCoralStationDistance.in(Meters)));
    }

    public static Trigger shouldStowElevator(Supplier<CoralStationSide> side,
            Supplier<Pose2d> robotPoseSupplier) {
        return new Trigger(
                () -> (FieldPoses.getDistanceFromRobotPose(FieldPoses.getCoralStationPose(side),
                        robotPoseSupplier).get() > FIELD_OFFSETS.elevatorPrepCoralStationDistance.in(Meters)));
    }
}