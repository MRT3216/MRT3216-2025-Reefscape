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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Coral.Positions;
import frc.robot.settings.Constants.ReefBranch;
import frc.robot.settings.RobotMap;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral.CoralPivotSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

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

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final CoralPivotSubsystem coralPivot = new CoralPivotSubsystem();

    /* Path follower */
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("None");;

    // #endregion

    public RobotContainer() {
        configureAutos();
        configureBindings();
    }

    private void configureAutos() {
        autoChooser.addOption("Left 3P", drivetrain.getLeft3PAuto());
        autoChooser.addOption("Center 1P", drivetrain.getCenter1PAuto());
        autoChooser.addOption("Right 3P", drivetrain.getRight3PAuto());

        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive
                                .withVelocityX(
                                        -driverController
                                                .getLeftY()
                                                * Constants.DRIVETRAIN.MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(
                                        -driverController
                                                .getLeftX()
                                                * Constants.DRIVETRAIN.MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(
                                        -driverController
                                                .getRightX()
                                                * Constants.DRIVETRAIN.MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(
        //         () -> point.withModuleDirection(
        //                 new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        // driverController.pov(0).whileTrue(
        //                 drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        // driverController.pov(180)
        //                 .whileTrue(drivetrain.applyRequest(
        //                                 () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

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

        // reset the field-centric heading on start press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.leftTrigger().whileTrue(drivetrain.driveToNearestLeftReefPole());
        driverController.rightTrigger().whileTrue(drivetrain.driveToNearestRightReefPole());
        driverController.rightBumper().whileTrue(drivetrain.driveAndAlignToReefBranch(ReefBranch.A));
        driverController.a().whileTrue(drivetrain.driveToLeftCoralStation());
        driverController.b().whileTrue(drivetrain.driveToRightCoralStation());
        driverController.x().whileTrue(drivetrain.driveToBargeClimb());
        driverController.y().whileTrue(drivetrain.driveToProcessor());

        driverController.povUp()
                .onTrue(elevator.moveElevatorToHeight(Positions.L4.getHeight())
                        .alongWith(coralPivot.movePivotToAngle(Positions.L4.getAngle())));
        driverController.povRight()
                .onTrue(elevator.moveElevatorToHeight(Positions.L3.getHeight())
                        .alongWith(coralPivot.movePivotToAngle(Positions.L3.getAngle())));
        driverController.povLeft()
                .onTrue(elevator.moveElevatorToHeight(Positions.L2.getHeight())
                        .alongWith(coralPivot.movePivotToAngle(Positions.L2.getAngle())));
        driverController.povDown()
                .onTrue(elevator.moveElevatorToHeight(Positions.L1.getHeight())
                        .alongWith(coralPivot.movePivotToAngle(Positions.L1.getAngle())));
        driverController.leftStick()
                .onTrue(elevator.moveElevatorToHeight(Positions.STARTING.getHeight())
                        .alongWith(coralPivot.movePivotToAngle(Positions.STARTING.getAngle())));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
    * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be
    * called on robot disable to prevent integral windup.
    */
    public void disablePIDSubsystems() {
        elevator.disable();
        coralPivot.disable();
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}