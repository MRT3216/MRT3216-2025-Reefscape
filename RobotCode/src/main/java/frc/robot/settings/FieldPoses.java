package frc.robot.settings;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.settings.Constants.BranchSide;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.settings.Constants.ReefBranch;

public class FieldPoses {
    public static Supplier<Pose2d> getNearestReefFaceInitial(Supplier<BranchSide> side, Supplier<Pose2d> currentPose) {
        return () -> getNearestReefFace(currentPose).get()
                .transformBy(Constants.FIELD_OFFSETS.getReefOffsetPoseInitial(side));
    }

    public static Supplier<Pose2d> getNearestReefFace(Supplier<Pose2d> currentPose) {
        // Alliance flip the robot pose, find the nearest blue side reef, then reflip
        Pose2d pose = AllianceFlipUtil.apply(currentPose.get())
                .nearest(Arrays.asList(FieldConstants.Reef.centerFaces));
        return () -> AllianceFlipUtil.apply(pose);
    }

    public static Supplier<Pose2d> getProcessorPose() {
        // Alliance flip the pose
        return () -> AllianceFlipUtil.apply(
                FieldConstants.Processor.centerFace.transformBy(Constants.FIELD_OFFSETS.processorOffset));
    }

    // public static Pose2d getBargePose(BargeCage cage) {
    //     Translation2d bargeTranslation = new Translation2d();

    //     switch (cage) {
    //         case farCage:
    //             bargeTranslation = FieldConstants.Barge.farCage;
    //             break;
    //         case middleCage:
    //             bargeTranslation = FieldConstants.Barge.middleCage;
    //             break;
    //         case closeCage:
    //             bargeTranslation = FieldConstants.Barge.closeCage;
    //     }

    //     // Alliance flip the pose
    //     return AllianceFlipUtil.apply(
    //             new Pose2d(bargeTranslation, Rotation2d.fromDegrees(0))
    //                     .transformBy(Constants.FIELD_OFFSETS.cageOffset));
    // }

    public static Supplier<Pose2d> getCoralStationPose(Supplier<CoralStationSide> side) {
        return () -> AllianceFlipUtil.apply(
                side.get().equals(CoralStationSide.LEFT) ? FieldConstants.CoralStation.leftCenterFace
                        : FieldConstants.CoralStation.rightCenterFace)
                .transformBy(Constants.FIELD_OFFSETS.coralStationOffsetPose);
    }

    public static Supplier<Pose2d> getReefPolePose(Supplier<ReefBranch> reefBranch) {
        return () -> AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[reefBranch.get().getReefSide().getValue()]
                .transformBy(Constants.FIELD_OFFSETS.getReefOffsetPoseInitial(() -> reefBranch.get().getBranchSide())));
    }

    public static Supplier<Double> getDistanceFromRobotPose(Supplier<Pose2d> pose, Supplier<Pose2d> currentPose) {
        return ()-> PhotonUtils.getDistanceToPose(currentPose.get(), pose.get());
    }

    /**
    * Calculates a target pose relative to an AprilTag on the field.
    *
    * @param aprilTag    The ID of the AprilTag.
    * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
    *                    itself correctly.
    * @return The target pose of the AprilTag.
    */
    public static Pose2d getAprilTagPoseFromFieldMap(int aprilTagID, Transform2d robotOffset) {
        Optional<Pose3d> aprilTagPose3d = FieldConstants.fieldLayout.getTagPose(aprilTagID);
        if (aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
        } else {
            throw new RuntimeException(
                    "Cannot get AprilTag " + aprilTagID + " from field " + FieldConstants.fieldLayout.toString());
        }
    }
}