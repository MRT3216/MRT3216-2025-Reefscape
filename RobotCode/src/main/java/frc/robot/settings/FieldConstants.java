// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.settings;

import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
            AprilTagFields.k2025ReefscapeWelded);
    public static final double fieldLength = fieldLayout.getFieldLength();
    public static final double fieldWidth = fieldLayout.getFieldWidth();
    public static final double startingLineX = Units.inchesToMeters(299.438); // Measured from the inside of starting line
    public static final double algaeDiameter = Units.inchesToMeters(16);

    public static class Processor {
        public static final Pose2d centerFace = new Pose2d(
                fieldLayout.getTagPose(16).get().getX(),
                0,
                Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d farCage = new Translation2d(Units.inchesToMeters(345.428),
                Units.inchesToMeters(286.779));
        public static final Translation2d middleCage = new Translation2d(Units.inchesToMeters(345.428),
                Units.inchesToMeters(242.855));
        public static final Translation2d closeCage = new Translation2d(Units.inchesToMeters(345.428),
                Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final double stationLength = Units.inchesToMeters(79.750);
        public static final Pose2d rightCenterFace = new Pose2d(
                Units.inchesToMeters(33.526),
                Units.inchesToMeters(25.824),
                Rotation2d.fromDegrees(144.011 - 90));
        public static final Pose2d leftCenterFace = new Pose2d(
                rightCenterFace.getX(),
                fieldWidth - rightCenterFace.getY(),
                Rotation2d.fromRadians(-rightCenterFace.getRotation().getRadians()));
    }

    public static class Reef {
        public static final double faceLength = Units.inchesToMeters(36.792600);
        public static final Translation2d center = new Translation2d(Units.inchesToMeters(176.746),
                fieldWidth / 2.0);
        public static final double faceToZoneLine = Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

        public static final Pose2d[] centerFaces = new Pose2d[6]; // Starting facing the driver station in clockwise order

        static {
            // Initialize faces
            centerFaces[0] = fieldLayout.getTagPose(18).get().toPose2d();
            centerFaces[1] = fieldLayout.getTagPose(19).get().toPose2d();
            centerFaces[2] = fieldLayout.getTagPose(20).get().toPose2d();
            centerFaces[3] = fieldLayout.getTagPose(21).get().toPose2d();
            centerFaces[4] = fieldLayout.getTagPose(22).get().toPose2d();
            centerFaces[5] = fieldLayout.getTagPose(17).get().toPose2d();
        }
    }

    public static class StagingPositions {
        // Measured from the center of the ice cream
        public static final double separation = Units.inchesToMeters(72.0);
        public static final Pose2d middleIceCream = new Pose2d(Units.inchesToMeters(48), fieldWidth / 2.0,
                new Rotation2d());
        public static final Pose2d leftIceCream = new Pose2d(Units.inchesToMeters(48),
                middleIceCream.getY() + separation, new Rotation2d());
        public static final Pose2d rightIceCream = new Pose2d(Units.inchesToMeters(48),
                middleIceCream.getY() - separation, new Rotation2d());
    }

    public enum ReefLevel {
        L1(Units.inchesToMeters(25.0), 0),
        L2(Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
        L3(Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
        L4(Units.inchesToMeters(72), -90);

        ReefLevel(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // Degrees
        }

        public static ReefLevel fromLevel(int level) {
            return Arrays.stream(values())
                    .filter(height -> height.ordinal() == level)
                    .findFirst()
                    .orElse(L4);
        }

        public final double height;
        public final double pitch;
    }

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
    public static final int aprilTagCount = 22;
}