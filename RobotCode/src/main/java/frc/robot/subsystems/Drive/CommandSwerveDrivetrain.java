package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.BargeCage;
import frc.robot.settings.Constants.BranchSide;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.settings.Constants.ReefBranch;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    // #region Fields

    /**
     * PhotonVision class to keep an accurate odometry.
     */
    public Vision vision;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /** Field object. */
    private Field2d field = new Field2d();

    private boolean slowMode = false;

    // #endregion

    // #region SysId

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    // #endregion

    // #region Constructors

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        setupPhotonVision();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>S
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        setupPhotonVision();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        setupPhotonVision();
    }

    // #endregion

    // #region Commands

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    private Command driveToPose(Pose2d pose) {
        return vision.getDistanceFromRobotPose(pose) > Constants.PATHING.pathingMinimumDistance
                ? AutoBuilder.pathfindToPose(pose, Constants.PATHING.pathConstraints, 0)
                : new DriveToPose(this, pose);
    }

    public Command driveToBargeClimb() {
        return this.defer(
                () -> this.driveToPose(
                        this.vision.getBargePose(BargeCage.middleCage)));
    }

    public Command driveToProcessor() {
        return this.defer(
                () -> this.driveToPose(
                        this.vision.getProcessorPose()));
    }

    public Command driveToLeftCoralStation() {
        return this.defer(
                () -> this.driveToPose(
                        vision.getCoralStationPose(CoralStationSide.LEFT)));
    }

    public Command driveToRightCoralStation() {
        return this.defer(
                () -> this.driveToPose(
                        vision.getCoralStationPose(CoralStationSide.RIGHT)));
    }

    public Command driveToNearestLeftReefPole() {
        return this.defer(() -> driveToNearestReefThenAlign(BranchSide.LEFT));
    }

    public Command driveToNearestRightReefPole() {
        return this.defer(() -> driveToNearestReefThenAlign(BranchSide.RIGHT));
    }

    private Command driveToNearestReefThenAlign(BranchSide side) {
        Pose2d reefPose = vision.getNearestReefFaceInitial(side);

        return driveToReefPoseThenAlign(reefPose);
    }

    private Command driveToReefPoseThenAlign(Pose2d reefPose) {
        Pose2d reefPoseClose = reefPose.transformBy(
                Constants.FIELD_OFFSETS.getReefOffsetPositionClose());

        System.out.println(vision.getDistanceFromRobotPose(reefPose));

        if (vision.getDistanceFromRobotPose(reefPose) < Constants.PATHING.pathingMinimumDistance) {
            System.out.println("CLOSE");
            return new DriveToPose(this, reefPoseClose);
        } else {
            System.out.println("FAR");
            return AutoBuilder
                    .pathfindToPose(reefPose, Constants.PATHING.pathConstraints,
                            Constants.PATHING.pathToCloseAlignEndVelocityMPS)
                    .andThen(new DriveToPose(this, reefPoseClose));
        }
    }

    public Command driveAndAlignToReefBranch(ReefBranch reefBranch) {
        return this.defer(
                () -> this.driveToReefPoseThenAlign(
                        vision.getReefPolePose(reefBranch)));
    }

    public Command toggleSlowMode() {
        return this.defer(() -> this.runOnce(
                () -> slowMode = !slowMode));
    }

    public boolean isSlowMode() {
        return slowMode;
    }

    public Command getLeft3PAuto() {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return driveAndAlignToReefBranch(ReefBranch.J)
                .andThen(driveToLeftCoralStation())
                .andThen(driveAndAlignToReefBranch(ReefBranch.K))
                .andThen(driveToLeftCoralStation())
                .andThen(driveAndAlignToReefBranch(ReefBranch.L))
                .andThen(driveToLeftCoralStation());
    }

    public Command getCenter1PAuto() {
        return driveAndAlignToReefBranch(ReefBranch.H);
    }

    public Command getRight3PAuto() {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return driveAndAlignToReefBranch(ReefBranch.E)
                .andThen(driveToRightCoralStation())
                .andThen(driveAndAlignToReefBranch(ReefBranch.D))
                .andThen(driveToRightCoralStation())
                .andThen(driveAndAlignToReefBranch(ReefBranch.C))
                .andThen(driveToRightCoralStation());
    }

    // #endregion

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            Constants.PATHING.TRANSLATION_PID,
                            // PID constants for rotation
                            Constants.PATHING.ANGLE_PID),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
            // DO THIS AFTER CONFIGURATION OF YOUR DESIRED PATHFINDER
            PathfindingCommand.warmupCommand().schedule();
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    /**
    * Setup the photon vision class.
    */
    private void setupPhotonVision() {
        vision = new Vision(() -> getState().Pose, field);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
        vision.updatePoseEstimation(this);

        SmartDashboard.putBoolean("Slow Mode", slowMode);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }
}

// public LinearVelocity getVelocityMagnitude() {
//     ChassisSpeeds cs = getState().Speeds;
//     return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
// }

// // Credit to FRC3136 ORCA (Official Robot Constructors of Ashland)
// private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target) {
//     if (this.getVelocityMagnitude().in(MetersPerSecond) < 0.25) {
//         var diff = target.minus(this.getState().Pose).getTranslation();
//         return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();// .rotateBy(Rotation2d.k180deg);
//     }
//     return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
// }

// // Credit to FRC3136 ORCA (Official Robot Constructors of Ashland)
// private Command goToTargetPoseOrcaMethod(Pose2d targetPose) {
//     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
//             new Pose2d(getState().Pose.getTranslation(),
//                     getPathVelocityHeading(this.getState().Speeds, targetPose)),
//             targetPose);

//     if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
//         return Commands.print("Auto alignment too close to desired position to continue");
//     }

//     PathPlannerPath path = new PathPlannerPath(
//             waypoints,
//             Constants.PATHING.pathConstraints,
//             new IdealStartingState(this.getVelocityMagnitude(), this.getState().Pose.getRotation()),
//             new GoalEndState(0.0, targetPose.getRotation()));

//     path.preventFlipping = true;

//     return AutoBuilder.followPath(path);
// }