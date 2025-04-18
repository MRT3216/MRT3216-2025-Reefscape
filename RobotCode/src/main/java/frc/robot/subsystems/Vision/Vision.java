package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.settings.FieldConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision {
    /**
     * Photon Vision Simulation
     */
    public VisionSystemSim visionSim;

    /**
     * Current pose from the pose estimator using wheel odometry.
     */
    private Supplier<Pose2d> currentPose;

    /**
     * Constructor for the Vision class.
     *
     * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
     * @param field       Current field, should be {@link SwerveDrive#field}
     */
    public Vision(Supplier<Pose2d> currentPose) {
        this.currentPose = currentPose;

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("Vision");
            visionSim.addAprilTags(FieldConstants.fieldLayout);

            for (Cameras c : Cameras.values()) {
                c.addToVisionSim(visionSim);
            }
        }
    }

    /**
     * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
     *
     * @param swerveDrive {@link SwerveDrive} instance.
     */
    public void updatePoseEstimation(CommandSwerveDrivetrain swerveDrive) {
        // if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
        //     /*
        //      * In the maple-sim, odometry is simulated using encoder values, accounting for factors like skidding and drifting.
        //      * As a result, the odometry may not always be 100% accurate.
        //      * However, the vision system should be able to provide a reasonably accurate pose estimation, even when odometry is incorrect.
        //      * (This is why teams implement vision system to correct odometry.)
        //      * Therefore, we must ensure that the actual robot pose is provided in the simulator when updating the vision simulation during the simulation.
        //      */
        //     visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
        // }
        if (Robot.isSimulation()) {
            visionSim.update(currentPose.get());
        }

        SmartDashboard.putNumber("Heading Pigeon", swerveDrive.getPigeon2().getRotation2d().getDegrees());
        SmartDashboard.putNumber("Heading States", swerveDrive.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putString("FR Strategy", Cameras.FRONT_RIGHT.getPrimaryStrategy().toString());
        SmartDashboard.putString("FL Strategy", Cameras.FRONT_LEFT.getPrimaryStrategy().toString());
        SmartDashboard.putBoolean("BR Enabled", Cameras.BACK_RIGHT.isEnabled().getAsBoolean());
        SmartDashboard.putBoolean("BL Enabled", Cameras.BACK_LEFT.isEnabled().getAsBoolean());

        if (DriverStation.isDisabled()) {
            //gyroOffset = new Rotation2d(0);
            //  swerveDrive.getPigeon2().getRotation2d()
            //         .minus(swerveDrive.getState().Pose.getRotation());
        }

        for (Cameras camera : Cameras.values()) {
            if (camera.getPrimaryStrategy() == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE ||
                    camera.getPrimaryStrategy() == PoseStrategy.CONSTRAINED_SOLVEPNP) {
                // TODO: Probably not right
                //camera.addHeadingData(swerveDrive.getPigeon2().getRotation2d().plus(gyroOffset));
                camera.addHeadingData(swerveDrive.getRobotPose().get().getRotation());
            }

            Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);

            if (poseEst.isPresent()) {
                var pose = poseEst.get();

                swerveDrive.addVisionMeasurement(
                        pose.estimatedPose.toPose2d(),
                        pose.timestampSeconds,
                        camera.curStdDevs);
            }
        }
    }

    /**
     * Generates the estimated robot pose. Returns empty if:
     * <ul>
     * <li> No Pose Estimates could be generated</li>
     * <li> The generated pose estimate was considered not accurate</li>
     * </ul>
     *
     * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
        Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
        if (Robot.isSimulation()) {
            Field2d debugField = visionSim.getDebugField();
            // Uncomment to enable outputting of vision targets in sim.
            poseEst.ifPresentOrElse(
                    est -> debugField
                            .getObject("VisionEstimation")
                            .setPose(est.estimatedPose.toPose2d()),
                    () -> {
                        debugField.getObject("VisionEstimation").setPoses();
                    });
        }
        return poseEst;
    }

    /**
     * Vision simulation.
     *
     * @return Vision Simulation
     */
    public VisionSystemSim getVisionSim() {
        return visionSim;
    }

    public void configureFarStrategy() {
        Cameras.BACK_LEFT.setEnabled(true);
        Cameras.BACK_RIGHT.setEnabled(true);

        Cameras.FRONT_LEFT.setStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        Cameras.FRONT_RIGHT.setStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    }

    public void configureCloseStrategy() {
        Cameras.BACK_LEFT.setEnabled(false);
        Cameras.BACK_RIGHT.setEnabled(false);

        Cameras.FRONT_LEFT.setStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
        Cameras.FRONT_RIGHT.setStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
        // Cameras.FRONT_LEFT.setStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        // Cameras.FRONT_RIGHT.setStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    }

    /**
     * Camera Enum to select each camera
     */
    enum Cameras {
        /**
        * Center Camera
        */
        FRONT_LEFT("Front-Left",
                new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(-15),
                        Units.degreesToRadians(-15)),
                new Translation3d(
                        Units.inchesToMeters(9.162306),
                        Units.inchesToMeters(10.001698),
                        Units.inchesToMeters(9.938572)),
                // Trig Std Devs
                VecBuilder.fill(4, 4, Double.MAX_VALUE),
                // Single-Tag Std Devs
                VecBuilder.fill(4, 4, 8),
                // Multi-Tag Std Devs
                VecBuilder.fill(0.5, 0.5, 1),
                new CameraIntrinsics(
                        1600, 1304,
                        1378.4945518084592, 1374.6731238827526, 776.2109098410699, 701.3215441831921,
                        new double[] { -0.030218242577181042, 0.010924281226336615, -2.9036984551012926E-4,
                                2.3696076486885392E-4, 0.019186086831858823, 0.0022537742416706484,
                                0.0012685606724970187, -9.966123793520036E-4 })),
        FRONT_RIGHT("Front-Right",
                new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(-15),
                        Units.degreesToRadians(15)),
                new Translation3d(
                        Units.inchesToMeters(9.162306),
                        Units.inchesToMeters(-10.001698),
                        Units.inchesToMeters(9.938572)),
                // Trig Std Devs
                VecBuilder.fill(4, 4, Double.MAX_VALUE),
                // Single-Tag Std Devs
                VecBuilder.fill(4, 4, 8),
                // Multi-Tag Std Devs
                VecBuilder.fill(0.5, 0.5, 1),
                new CameraIntrinsics(
                        1600, 1304,
                        1374.5414918415538, 1371.4502669827698, 757.1200966271149, 650.6543370498349,
                        new double[] { -0.02599220352443215, -0.004255449108769354, -0.0010584158777595613,
                                3.151541785665642E-4, 0.035143858610829005, 0.0010261268177625253, 2.102376068279724E-4,
                                -0.0013940898120434317 })),
        BACK_LEFT("Back-Left",
                new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(-5),
                        Units.degreesToRadians(160)),
                new Translation3d(
                        Units.inchesToMeters(3),
                        Units.inchesToMeters(11.947),
                        Units.inchesToMeters(26.125)),
                // Trig Std Devs
                VecBuilder.fill(4, 4, Double.MAX_VALUE),
                // Single-Tag Std Devs
                VecBuilder.fill(4, 4, 8),
                // Multi-Tag Std Devs
                VecBuilder.fill(0.5, 0.5, 1),
                new CameraIntrinsics(
                        1280, 800,
                        906.8650501556826, 904.8249218594106, 685.0939074656633, 440.3743360464017,
                        new double[] { 0.048427656762511505, -0.07279893955862766, -2.404548497778298E-4,
                                2.922420290616374E-4, 0.01566490569651964, -0.002543695016824542, 0.00631510120502211,
                                0.0019736324419207425 })),
        BACK_RIGHT("Back-Right",
                new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(-5),
                        Units.degreesToRadians(200)),
                new Translation3d(
                        Units.inchesToMeters(3),
                        Units.inchesToMeters(-11.947),
                        Units.inchesToMeters(26.125)),
                // Trig Std Devs
                VecBuilder.fill(1, 1, Double.MAX_VALUE),
                // Single-Tag Std Devs
                VecBuilder.fill(4, 4, 8),
                // Multi-Tag Std Devs
                VecBuilder.fill(0.5, 0.5, 1),
                new CameraIntrinsics(
                        1280, 800,
                        905.165459096268, 903.4404452296051, 640.6433787059736, 412.999885030855,
                        new double[] { 0.05018289179599694, -0.06965163455567655, -3.9595872419393856E-4,
                                1.4575235566772828E-4, 0.011609663736452296, -0.0024169860307781403,
                                0.006570847209678644, 0.0019650281377058872 }));

        /**
         * Latency alert to use when high latency is detected.
         */
        public final Alert latencyAlert;
        /**
         * Camera instance for comms.
         */
        public final PhotonCamera camera;
        /**
         * Pose estimator for camera.
         */
        public final PhotonPoseEstimator poseEstimator;
        /**
        * Standard Deviation for trig readings for pose estimation.
        */
        private final Matrix<N3, N1> trigStdDevs;
        /**
         * Standard Deviation for single tag readings for pose estimation.
         */
        private final Matrix<N3, N1> singleTagStdDevs;
        /**
         * Standard deviation for multi-tag readings for pose estimation.
         */
        private final Matrix<N3, N1> multiTagStdDevs;
        /**
         * Transform of the camera rotation and translation relative to the center of the robot
         */
        private final Transform3d robotToCamTransform;
        /**
         * Current standard deviations used.
         */
        public Matrix<N3, N1> curStdDevs;
        /**
         * Estimated robot pose.
         */
        public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();;
        /**
         * Simulated camera instance which only exists during simulations.
         */
        public PhotonCameraSim cameraSim;
        /**
         * Results list to be updated periodically and cached to avoid unnecessary queries.
         */
        public List<PhotonPipelineResult> resultsList = new ArrayList<>();

        private boolean enabled = true;

        /**
         * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
         * estimation noise on an actual robot.
         *
         * @param name                  Name of the PhotonVision camera found in the PV UI.
         * @param robotToCamRotation    {@link Rotation3d} of the camera.
         * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
         * @param singleTagStdDevs      Single AprilTag standard deviations of estimated poses from the camera.
         * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
         */
        Cameras(String name,
                Rotation3d robotToCamRotation,
                Translation3d robotToCamTranslation,
                Matrix<N3, N1> trigStdDevs,
                Matrix<N3, N1> singleTagStdDevs,
                Matrix<N3, N1> multiTagStdDevsMatrix,
                CameraIntrinsics intrinsics) {
            latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

            camera = new PhotonCamera(name);

            // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
            robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

            poseEstimator = new PhotonPoseEstimator(FieldConstants.fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCamTransform);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            this.trigStdDevs = trigStdDevs;
            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevsMatrix;

            if (Robot.isSimulation()) {
                SimCameraProperties cameraProp = new SimCameraProperties();
                // A 640 x 480 camera with a 100 degree diagonal FOV.
                cameraProp.setCalibration(intrinsics.resX, intrinsics.resY, intrinsics.getCameraMatrix(),
                        intrinsics.getDistCoeffs());
                // Approximate detection noise with average and standard deviation error in pixels.
                cameraProp.setCalibError(0.25, 0.08);
                // Set the camera image capture framerate (Note: this is limited by robot loop rate).
                cameraProp.setFPS(30);
                // The average and standard deviation in milliseconds of image data latency.
                cameraProp.setAvgLatencyMs(35);
                cameraProp.setLatencyStdDevMs(5);

                cameraSim = new PhotonCameraSim(camera, cameraProp);

                cameraSim.enableDrawWireframe(true);
            }
        }

        public BooleanSupplier isEnabled() {
            return () -> enabled;
        }

        public void setEnabled(boolean enabled) {
            this.enabled = enabled;
        }

        public void setStrategy(PoseStrategy strategy) {
            poseEstimator.setPrimaryStrategy(strategy);
        }

        public PoseStrategy getPrimaryStrategy() {
            return poseEstimator.getPrimaryStrategy();
        }

        /**
         * Add camera to {@link VisionSystemSim} for simulated photon vision.
         *
         * @param systemSim {@link VisionSystemSim} to use.
         */
        public void addToVisionSim(VisionSystemSim systemSim) {
            if (Robot.isSimulation()) {
                systemSim.addCamera(cameraSim, robotToCamTransform);
            }
        }

        /**
        * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the
        * cache of results.
        *
        * @return Estimated pose.
        */
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
            updateUnreadResults();
            return estimatedRobotPose;
        }

        /**
        * Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by timestamp.
        */
        private void updateUnreadResults() {
            resultsList = Robot.isReal() ? camera.getAllUnreadResults()
                    : cameraSim.getCamera().getAllUnreadResults();

            resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
                return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
            });
            if (!resultsList.isEmpty()) {
                updateEstimatedGlobalPose();
            }
        }

        /**
         * The latest estimated robot pose on the field from vision data. This may be empty. This should only be called once
         * per loop.
         *
         * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
         * {@link Cameras#updateEstimationStdDevs}
         *
         * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets used for
         * estimation.
         */
        private void updateEstimatedGlobalPose() {
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            if (this.enabled) {
                for (var change : resultsList) {
                    if (poseEstimator.getPrimaryStrategy() == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE ||
                            poseEstimator.getPrimaryStrategy() == PoseStrategy.CONSTRAINED_SOLVEPNP) {

                        boolean headingFree = DriverStation.isDisabled();
                        var constrainedPnpParams = new PhotonPoseEstimator.ConstrainedSolvepnpParams(headingFree, 1.0);

                        visionEst = poseEstimator.update(change, camera.getCameraMatrix(), camera.getDistCoeffs(),
                                Optional.of(constrainedPnpParams));
                    } else {
                        visionEst = poseEstimator.update(change);
                    }

                    updateEstimationStdDevs(visionEst, change.getTargets());
                }
            }
            estimatedRobotPose = visionEst;
        }

        public void addHeadingData(Rotation2d heading) {
            poseEstimator.addHeadingData(Timer.getFPGATimestamp(), heading);
        }

        /**
         * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
         * on number of tags, estimation strategy, and distance from the tags.
         *
         * @param estimatedPose The estimated pose to guess standard deviations for.
         * @param targets       All targets in this camera frame
         */
        private void updateEstimationStdDevs(
                Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

            boolean usingTrig = poseEstimator.getPrimaryStrategy() == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;

            if (estimatedPose.isEmpty()) {
                // No pose input. Default to single-tag std devs
                curStdDevs = singleTagStdDevs;
            } else {
                // Pose present. Start running Heuristic
                var estStdDevs = usingTrig ? trigStdDevs : singleTagStdDevs;
                int numTags = 0;
                double avgDist = 0;

                // Precalculation - see how many tags we found, and calculate an average-distance metric
                for (var tgt : targets) {
                    var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                    if (tagPose.isEmpty()) {
                        continue;
                    }
                    // Get the target AprilTag, and reject the measurement if the
                    // tag is not configured to be utilized by the pose estimator.
                    if (usingTrig) {
                        int id = tgt.fiducialId;
                        if (!isReefTag(id))
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
                    curStdDevs = singleTagStdDevs;
                } else {
                    // One or more tags visible, run the full heuristic.
                    avgDist /= numTags;
                    // Decrease std devs if multiple targets are visible
                    if (numTags > 1) {
                        estStdDevs = multiTagStdDevs;
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
        }

        /**
        * Returns {@code true} if an AprilTag should be utilized.
        * @param id The ID of the AprilTag.
        */
        private boolean isReefTag(int id) {
            return DriverStation.getAlliance().get() == Alliance.Blue ? (id >= 17 && id <= 22) : (id >= 6 && id <= 11);
        }
    }
}