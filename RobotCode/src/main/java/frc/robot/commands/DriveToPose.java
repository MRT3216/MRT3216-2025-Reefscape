package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DriveToPose extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final ProfiledPIDController m_translationController, m_thetaController;
    private Translation2d m_lastSetpointTranslation;
    private Pose2d m_goalPose;
    private SwerveRequest.ApplyFieldSpeeds m_drive;

    public DriveToPose(CommandSwerveDrivetrain drivetrain, Pose2d goalPose) {
        m_goalPose = goalPose;
        m_translationController = new ProfiledPIDController(
                Constants.CLOSE_PATHING.TRANSLATION_PID.kP,
                Constants.CLOSE_PATHING.TRANSLATION_PID.kI,
                Constants.CLOSE_PATHING.TRANSLATION_PID.kD,
                new TrapezoidProfile.Constraints(
                        Constants.CLOSE_PATHING.maxVelocityMPS,
                        Constants.CLOSE_PATHING.maxAccelerationMPSSq));
        m_thetaController = new ProfiledPIDController(
                Constants.CLOSE_PATHING.ANGLE_PID.kP,
                Constants.CLOSE_PATHING.ANGLE_PID.kP,
                Constants.CLOSE_PATHING.ANGLE_PID.kP,
                new TrapezoidProfile.Constraints(
                        Constants.CLOSE_PATHING.maxAngularVelocityRPS,
                        Constants.CLOSE_PATHING.maxAngularAccelerationRPS));
        this.m_drivetrain = drivetrain;

        m_drive = new SwerveRequest.ApplyFieldSpeeds()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d initialPose = m_drivetrain.getState().Pose;

        m_translationController.setTolerance(0.05);
        m_thetaController.setTolerance(Units.degreesToRadians(1.0));

        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        ChassisSpeeds robotVelocity = getFieldRelativeChassisSpeeds(m_drivetrain.getState().Speeds,
                initialPose);

        m_translationController.reset(
                initialPose.getTranslation().getDistance(m_goalPose.getTranslation()),
                Math.min(
                        0.0,
                        -new Translation2d(robotVelocity.vxMetersPerSecond,
                                robotVelocity.vyMetersPerSecond)
                                .rotateBy(
                                        m_goalPose
                                                .getTranslation()
                                                .minus(initialPose
                                                        .getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));

        m_thetaController.reset(
                initialPose.getRotation().getRadians(),
                robotVelocity.omegaRadiansPerSecond);
        m_lastSetpointTranslation = initialPose.getTranslation();
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drivetrain.getState().Pose;

        double distanceToGoalPose = currentPose.getTranslation().getDistance(m_goalPose.getTranslation());

        double ffScaler = MathUtil.clamp((distanceToGoalPose - 0.2) / (0.8 - 0.2), 0.0, 1.0);

        m_translationController.reset(
                m_lastSetpointTranslation.getDistance(m_goalPose.getTranslation()),
                m_translationController.getSetpoint().velocity);//ROBO RELATIVE

        double driveVelocityScalar = m_translationController.getSetpoint().velocity * ffScaler
                + m_translationController.calculate(distanceToGoalPose, 0.0);

        if (distanceToGoalPose < m_translationController.getPositionTolerance())
            driveVelocityScalar = 0.0;

        m_lastSetpointTranslation = new Pose2d(
                m_goalPose.getTranslation(),
                currentPose.getTranslation().minus(m_goalPose.getTranslation()).getAngle())
                .transformBy(
                        new Transform2d(
                                new Translation2d(
                                        m_translationController
                                                .getSetpoint().position,
                                        0.0),
                                new Rotation2d()))
                .getTranslation();

        double thetaVelocity = m_thetaController.getSetpoint().velocity * ffScaler
                + m_thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        m_goalPose.getRotation().getRadians());
        double thetaErrorAbsolute = Math
                .abs(currentPose.getRotation().minus(m_goalPose.getRotation()).getRadians());
        if (thetaErrorAbsolute < m_thetaController.getPositionTolerance())
            thetaVelocity = 0.0;

        Translation2d driveVelocity = new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(m_goalPose.getTranslation()).getAngle())
                .transformBy(
                        new Transform2d(new Translation2d(driveVelocityScalar, 0.0),
                                new Rotation2d()))
                .getTranslation();

        final ChassisSpeeds CS = new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity);

        m_drivetrain.setControl(m_drive.withSpeeds(CS));
    }

    @Override
    public boolean isFinished() {
        return m_translationController.atGoal() && m_thetaController.atGoal();
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds(ChassisSpeeds roboSpeed, Pose2d pose) {
        return new ChassisSpeeds(
                roboSpeed.vxMetersPerSecond * pose.getRotation().getCos()
                        - roboSpeed.vyMetersPerSecond * pose.getRotation().getSin(),
                roboSpeed.vyMetersPerSecond * pose.getRotation().getCos()
                        + roboSpeed.vxMetersPerSecond * pose.getRotation().getSin(),
                roboSpeed.omegaRadiansPerSecond);
    }
}