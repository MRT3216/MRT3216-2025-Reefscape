// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.settings.FieldConstants;
import frc.robot.settings.VisionConstants;
import frc.robot.settings.VisionConstants.CamerasConstants;

public class Camera extends SubsystemBase {
    // Save the previous result for reference pose strategy
    Pose2d savedResult = new Pose2d(0, 0, new Rotation2d(0.01, 0.01));

    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;
    private CommandSwerveDrivetrain driveTrain;
    /**
    * Simulated camera instance which only exists during simulations.
    */
    public PhotonCameraSim cameraSim;
    public VisionSystemSim visionSim;

    // Whether the camera should be used for vision measurements
    private boolean isEnabled = true;

    /**
     * Creates a new Camera.
     * @param cameraConstants
     * @param driveTrain
     * @param visionSim
     */
    public Camera(CamerasConstants cameraConstants, CommandSwerveDrivetrain driveTrain, VisionSystemSim visionSim) {
        this.driveTrain = driveTrain;
        this.visionSim = visionSim;
        photonCamera = new PhotonCamera(cameraConstants.photonName);

        // Default strategy
        photonPoseEstimator = new PhotonPoseEstimator(
                FieldConstants.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraConstants.robotToCamTransform);

        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Robot.isSimulation()) {
            SimCameraProperties cameraProp = new SimCameraProperties();

            cameraProp.setCalibration(cameraConstants.intrinsics.resX, cameraConstants.intrinsics.resY,
                    cameraConstants.intrinsics.getCameraMatrix(),
                    cameraConstants.intrinsics.getDistCoeffs());
            // Approximate detection noise with average and standard deviation error in pixels.
            cameraProp.setCalibError(0.25, 0.08);
            // Set the camera image capture framerate (Note: this is limited by robot loop rate).
            cameraProp.setFPS(30);
            // The average and standard deviation in milliseconds of image data latency.
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);

            cameraSim = new PhotonCameraSim(photonCamera, cameraProp);

            cameraSim.enableDrawWireframe(true);
            // Add the camera to the simulation
            visionSim.addCamera(cameraSim, cameraConstants.robotToCamTransform);
        }
    }

    /*
     * Returns the distance to the best target in meters.
     * @param pipelineResult The pipeline result to get the best target from.
     */
    public Double getBestTargetDistance(PhotonPipelineResult pipelineResult) {
        return FieldConstants.fieldLayout
                .getTagPose(pipelineResult.getBestTarget().getFiducialId())
                .get()
                .getTranslation()
                .getDistance(
                        new Translation3d(
                                driveTrain.getState().Pose.getX(), driveTrain.getState().Pose.getY(), 0.0));
    }

    /*
     * Returns the estimated robot pose from the given pipeline result.
     * @param previousRobotPose The previous robot pose to use as a reference.
     * @param pipelineResult The pipeline result to use for pose estimation.
     */
    public Optional<EstimatedRobotPose> getMultiTagPose3d(
            Pose2d previousRobotPose,
            PhotonPipelineResult pipelineResult) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        // Pass in the previous robot pose for reference pose strategy
        photonPoseEstimator.setReferencePose(previousRobotPose);

        if (photonPoseEstimator.getPrimaryStrategy() == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE ||
                photonPoseEstimator.getPrimaryStrategy() == PoseStrategy.CONSTRAINED_SOLVEPNP) {

            // Heading needs to be added for these strategies, but wait until robot is enabled
            boolean headingFree = DriverStation.isDisabled();
            var constrainedPnpParams = new PhotonPoseEstimator.ConstrainedSolvepnpParams(headingFree, 1.0);

            // Add the current pose of the robot as the heading
            addHeadingData(driveTrain.getRobotPose().get().getRotation());

            // Update the pose estimator with the pipeline result and camera intrinsics (these are pulled from NT)
            visionEst = photonPoseEstimator.update(
                    pipelineResult,
                    photonCamera.getCameraMatrix(),
                    photonCamera.getDistCoeffs(),
                    Optional.of(constrainedPnpParams));
        } else {
            // Update the pose estimator with the pipeline result
            visionEst = photonPoseEstimator.update(pipelineResult);
        }
        return visionEst;
    }

    /*
     * Returns the estimated robot pose from the given pipeline result.
     * @param pipelineResult The pipeline result to use for pose estimation.
     */
    public Pose2d getPose2d(PhotonPipelineResult pipelineResult) {
        Optional<EstimatedRobotPose> pose3d = getMultiTagPose3d(savedResult, pipelineResult);
        if (pose3d.isEmpty()) {
            return null;
        }
        savedResult = pose3d.get().estimatedPose.toPose2d();
        return savedResult;
    }

    /*
     * Returns {@code true} if the given pipeline result has a target.
     * @param pipeline The pipeline result to check for a target.
     */
    public boolean hasTarget(PhotonPipelineResult pipeline) {
        if (pipeline == null) {
            return false;
        }
        return pipeline.hasTargets();
    }

    /*
     * Returns a {@link BooleanSupplier} that returns whether the camera is enabled.
     */
    public BooleanSupplier isEnabled() {
        return () -> this.isEnabled;
    }

    /*
     * Sets whether the camera is enabled.
     * @param isEnabled Whether the camera should be enabled.
     */
    public void setEnabled(boolean isEnabled) {
        this.isEnabled = isEnabled;
    }

    /*
     * Sets the primary strategy for the camera.
     * @param strategy The primary strategy to use.
     */
    public void setStrategy(PoseStrategy strategy) {
        photonPoseEstimator.setPrimaryStrategy(strategy);
    }

    /*  
     * Returns the primary strategy for the camera.
     */
    public PoseStrategy getPrimaryStrategy() {
        return photonPoseEstimator.getPrimaryStrategy();
    }

    /*
     * Returns the name of the Photon camera.
     */
    public String getPhotonName() {
        return photonCamera.getName();
    }

    /*
     * Adds the filtered pose to the drivetrain applying appropriate stddevs.
     */
    public void addFilteredPose() {
        // Get the latest results from the camera
        List<PhotonPipelineResult> resultsList = Robot.isReal() ? photonCamera.getAllUnreadResults()
                : cameraSim.getCamera().getAllUnreadResults();

        for (PhotonPipelineResult pipelineResult : resultsList) {
            if (pipelineResult != null && hasTarget(pipelineResult)) {
                // Get the estimated pose from the pipeline result
                Optional<EstimatedRobotPose> estPose = getMultiTagPose3d(driveTrain.getState().Pose, pipelineResult);
                this.addPoseToSim(estPose);

                List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
                // Start assuming a reef tag is visible
                boolean hasReefTag = true;
                for (PhotonTrackedTarget target : targets) {
                    if (!isReefTag(target.getFiducialId())) {
                        hasReefTag = false;
                    }
                }

                if (hasReefTag
                        || photonPoseEstimator.getPrimaryStrategy() == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {

                    Pose2d bestRobotPose2d = getPose2d(pipelineResult);

                    if (bestRobotPose2d != null) {
                        //Matrix<N3, N1> visionMatrix = getSpeedStdDev(pipelineResult);
                        Matrix<N3, N1> visionMatrix = getEstimationStdDevs(estPose, targets);

                        driveTrain.addVisionMeasurement(
                                bestRobotPose2d, pipelineResult.getTimestampSeconds(), visionMatrix);
                        SmartDashboard.putBoolean("VisionUsed", true);
                    }
                } else {
                    SmartDashboard.putBoolean("VisionUsed", false);
                }
            } else {

                SmartDashboard.putBoolean("VisionUsed", false);
            }
        }

        if (Robot.isSimulation()) {
            visionSim.update(driveTrain.getRobotPose().get());
        }
    }

    /*
     * Returns the standard deviations for the speed estimation.
     * This method uses a heuristic to determine the standard deviations based on the 
     * distance to the best target and the speed of the robot.
     * @param pipelineResult The pipeline result to get the best target from.
     * @return The standard deviations for the speed estimation.
     */
    private Matrix<N3, N1> getSpeedStdDev(PhotonPipelineResult pipelineResult) {
        double bestTargetDistance = getBestTargetDistance(pipelineResult);
        double speedMultiplier = 1;
        if (Math.sqrt(
                Math.pow(driveTrain.getState().Speeds.vxMetersPerSecond, 2)
                        + Math.pow(driveTrain.getState().Speeds.vyMetersPerSecond,
                                2)) > 0.5) {
            speedMultiplier = 2;
        }
        double xKalman = this.lerp((bestTargetDistance - 0.6) / 2.4, 0.05, 0.5) * speedMultiplier;
        double yKalman = this.lerp((bestTargetDistance - 0.6) / 2.4, 0.05, 0.5) * speedMultiplier;
        double rotationKalman = this.lerp((bestTargetDistance - 0.6) / 1.4, 0.4, 1000) / 10;

        // Increase the rotation stddev for trig strategies
        if (photonPoseEstimator.getPrimaryStrategy() == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE ||
                photonPoseEstimator.getPrimaryStrategy() == PoseStrategy.CONSTRAINED_SOLVEPNP) {
            rotationKalman *= 1000;
        }

        Matrix<N3, N1> visionMatrix = VecBuilder.fill(xKalman, yKalman, rotationKalman);

        return visionMatrix;
    }

    /*
     * Returns the standard deviations for the pose estimation.
     * This method uses a heuristic to determine the standard deviations based on the number of targets
     * and their distance from the robot.
     * 
     * @param estimatedPose The estimated robot pose.
     * @param targets The list of targets.
     * @return The standard deviations for the pose estimation.
     */
    private Matrix<N3, N1> getEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        Matrix<N3, N1> curStdDevs = new Matrix<>(VecBuilder.fill(10, 10, 10));

        boolean usingTrig = photonPoseEstimator.getPrimaryStrategy() == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;

        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.singleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = usingTrig ? VisionConstants.trigStdDevs : VisionConstants.singleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                }
                numTags++;

                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.singleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) {
                    estStdDevs = VisionConstants.multiTagStdDevs;
                }
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4) {
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                }
                curStdDevs = estStdDevs;
            }
        }

        return curStdDevs;
    }

    /*
     * Adds the pose to the simulation.
     * @param estPose The estimated robot pose.
     */
    public void addPoseToSim(Optional<EstimatedRobotPose> estPose) {
        if (Robot.isSimulation()) {
            Field2d debugField = visionSim.getDebugField();
            // Uncomment to enable outputting of vision targets in sim.
            estPose.ifPresentOrElse(
                    est -> debugField
                            .getObject("VisionEstimation")
                            .setPose(est.estimatedPose.toPose2d()),
                    () -> {
                        debugField.getObject("VisionEstimation").setPoses();
                    });
        } else {
            SmartDashboard.putString("Vision Pose", estPose.get().estimatedPose.toPose2d().toString());
        }
    }

    /*
     * Adds heading data to the pose estimator.
     * @param heading The heading to add.
     */
    public void addHeadingData(Rotation2d heading) {
        photonPoseEstimator.addHeadingData(Timer.getFPGATimestamp(), heading);
    }

    /**
    * Returns {@code true} if an AprilTag should be utilized.
    * @param id The ID of the AprilTag.
    */
    private boolean isReefTag(int id) {
        return DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == Alliance.Blue
                        ? (id >= 17 && id <= 22)
                        : (id >= 6 && id <= 11);
    }

    /*
     * Linearly interpolates between two values.
     * @param t The interpolation value.
     * @param a The first value.
     * @param b The second value.
     * @return The interpolated value.
     */
    public double lerp(double t, double a, double b) {
        t = Math.max(0, Math.min(1, t));

        return a + (t * (b - a));
    }
}