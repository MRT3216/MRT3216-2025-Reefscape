// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.settings.Constants;
import frc.robot.settings.FieldConstants;
import frc.robot.settings.OIUtils;
import frc.robot.settings.RobotMap;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    // #region Fields

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.DRIVETRAIN.MaxSpeed * Constants.OI.kJoystickDeadband)
            .withRotationalDeadband(Constants.DRIVETRAIN.MaxAngularRate * Constants.OI.kJoystickDeadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(Constants.DRIVETRAIN.MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(
            RobotMap.ROBOT.DRIVE_STATION.DRIVER_USB_XBOX_CONTROLLER);
    private final CommandXboxController operatorController = new CommandXboxController(
            RobotMap.ROBOT.DRIVE_STATION.OPERATOR_USB_XBOX_CONTROLLER);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    // #endregion

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive
                                .withVelocityX(
                                        OIUtils.modifyAxis(-driverController
                                                .getLeftY(),
                                                Constants.OI.kTranslationExpo)
                                                * Constants.DRIVETRAIN.MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(
                                        OIUtils.modifyAxis(-driverController
                                                .getLeftX(),
                                                Constants.OI.kTranslationExpo)
                                                * Constants.DRIVETRAIN.MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(
                                        OIUtils.modifyAxis(-driverController
                                                .getRightX(),
                                                Constants.OI.kTranslationExpo)
                                                * Constants.DRIVETRAIN.MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(
        //         () -> point.withModuleDirection(
        //                 new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        driverController.pov(0).whileTrue(
                drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        driverController.pov(180)
                .whileTrue(drivetrain.applyRequest(
                        () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.leftTrigger().whileTrue(
                Commands.deferredProxy(
                        () -> drivetrain.driveToNearestReefThenAlign(true)));

        driverController.rightTrigger().whileTrue(
                Commands.deferredProxy(
                        () -> drivetrain.driveToNearestReefThenAlign(false)));

        driverController.a().whileTrue(
                Commands.deferredProxy(
                        () -> drivetrain.driveToPose(FieldConstants.CoralStation.leftCenterFace.transformBy(
                                FieldConstants.CoralStation.coralStationOffsetPose))));

        driverController.b().whileTrue(
                Commands.deferredProxy(
                        () -> drivetrain.driveToPose(FieldConstants.CoralStation.rightCenterFace.transformBy(
                                FieldConstants.CoralStation.coralStationOffsetPose))));

        driverController.x().whileTrue(
                Commands.deferredProxy(
                        () -> drivetrain.driveToPose(
                                drivetrain.vision.getBargePose())));

        driverController.y().whileTrue(
                Commands.deferredProxy(
                        () -> drivetrain.driveToPose(
                                drivetrain.vision.getProcessorPose())));

        // driverController.y().whileTrue(
        //         Commands.deferredProxy(
        //                 () -> drivetrain.driveToPose(
        //                         drivetrain.vision.getOffsetPoseByTagId(17, false))));
        // driverController.rightBumper().whileTrue(
        //         Commands.deferredProxy(
        //                 () -> drivetrain.driveToPoseThenAlign(
        //                         drivetrain.vision.getOffsetPoseByTagId(17, false))));

        //driverController.leftBumper().whileTrue();

        //PhotonTrackedTarget target = drivetrain.vision.getTargetFromId(17, drivetrain.vision.Cameras.CENTER_CAM);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}