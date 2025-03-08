// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.CLIMBER;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.settings.RobotMap;
import frc.robot.subsystems.Algae.Pivot.AlgaePivotSubsystem;
import frc.robot.subsystems.Algae.Rollers.AlgaeRollersSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Coral.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Coral.EndEffector.CoralEndEffectorSubsystem;
import frc.robot.subsystems.Coral.Pivot.CoralPivotSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class RobotContainer {
    // #region Fields

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.DRIVETRAIN.MaxSpeed * Constants.OI.kJoystickDeadband)
            .withRotationalDeadband(Constants.DRIVETRAIN.MaxAngularRate * Constants.OI.kJoystickDeadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(Constants.DRIVETRAIN.MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(
            RobotMap.ROBOT.DRIVE_STATION.DRIVER_USB_XBOX_CONTROLLER);
    private final CommandXboxController operatorController = new CommandXboxController(
            RobotMap.ROBOT.DRIVE_STATION.OPERATOR_USB_XBOX_CONTROLLER);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final CoralPivotSubsystem coralPivot = new CoralPivotSubsystem();
    private final CoralEndEffectorSubsystem coralEndEffector = new CoralEndEffectorSubsystem();
    private final AlgaePivotSubsystem algaePivot = new AlgaePivotSubsystem();
    private final AlgaeRollersSubsystem algaeRollers = new AlgaeRollersSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();

    /* Path follower */
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("None");;

    // #endregion

    public RobotContainer() {
        configureAutos();
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureAutos() {
        autoChooser.addOption("Left 3P", drivetrain.getLeft3PAuto());
        autoChooser.addOption("Center 1P", drivetrain.getCenter1PAuto());
        autoChooser.addOption("Right 3P", drivetrain.getRight3PAuto());

        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureDriverBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive
                                .withVelocityX(
                                        -driverController
                                                .getLeftY()
                                                * Constants.DRIVETRAIN.MaxSpeed
                                                * (drivetrain.isSlowMode().getAsBoolean()
                                                        || elevator.aboveHeight().getAsBoolean()
                                                                ? 0.1
                                                                : 1.0)) // Drive forward with negative Y (forward)
                                .withVelocityY(
                                        -driverController
                                                .getLeftX()
                                                * Constants.DRIVETRAIN.MaxSpeed
                                                * (drivetrain.isSlowMode().getAsBoolean()
                                                        || elevator.aboveHeight().getAsBoolean()
                                                                ? 0.1
                                                                : 1.0)) // Drive left with negative X (left)
                                .withRotationalRate(
                                        -driverController
                                                .getRightX()
                                                * Constants.DRIVETRAIN.MaxAngularRate
                                                * (drivetrain.isSlowMode().getAsBoolean()
                                                        || elevator.aboveHeight().getAsBoolean()
                                                                ? 0.3
                                                                : 1.0)) // Drive counterclockwise with negative X (left)
                ));

        driverController.a().whileTrue(drivetrain.driveToLeftCoralStation());
        driverController.b().whileTrue(drivetrain.driveToRightCoralStation());
        driverController.x().whileTrue(drivetrain.driveToBargeClimb());
        driverController.y().whileTrue(drivetrain.driveToProcessor());
        driverController.leftTrigger().whileTrue(drivetrain.driveToNearestLeftReefPole());
        driverController.rightTrigger().whileTrue(drivetrain.driveToNearestRightReefPole());
        driverController.leftBumper().onTrue(algaePivot.togglePivotPosition());
        driverController.rightBumper().whileTrue(algaeRollers.runRollerCommand());
        // Reset the field-centric heading on start press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.rightStick().onTrue(drivetrain.toggleSlowMode());

        // TODO: Use this method to aim wheels for climb
        // driverController.b().whileTrue(drivetrain.applyRequest(
        //         () -> point.withModuleDirection(
        //                 new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y())
        //         .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x())
        //         .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverController.povUp()
                .onTrue(CoralCommands.moveElevatorAndPivotToHeight(elevator, coralPivot, POSITIONS.L4));
        driverController.povRight()
                .onTrue(CoralCommands.moveElevatorAndPivotToHeight(elevator, coralPivot, POSITIONS.L3));
        driverController.povLeft()
                .onTrue(CoralCommands.moveElevatorAndPivotToHeight(elevator, coralPivot, POSITIONS.L2));
        driverController.povDown()
                .onTrue(CoralCommands.moveElevatorAndPivotToHeight(elevator, coralPivot, POSITIONS.L1));
        driverController.back()
                .onTrue(CoralCommands.moveElevatorAndPivotToHeight(elevator, coralPivot, POSITIONS.SCORE_PREP));

        // driverController.leftBumper().onTrue(coralPivot.adjustPivotAngle(Degrees.of(-1)));
        // driverController.rightBumper().onTrue(coralPivot.adjustPivotAngle(Degrees.of(1)));

        // driverController.leftTrigger().onTrue(elevator.adjustElevatorHeight(Inches.of(-0.5)));
        // driverController.rightTrigger().onTrue(elevator.adjustElevatorHeight(Inches.of(0.5)));

        // driverController.a().whileTrue(coralEndEffector.runEndEffector());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureOperatorBindings() {
        operatorController.a().onTrue(elevator.setTargetPos(POSITIONS.L1));
        operatorController.b().onTrue(elevator.setTargetPos(POSITIONS.L2));
        operatorController.x().onTrue(elevator.setTargetPos(POSITIONS.L3));
        operatorController.y().onTrue(elevator.setTargetPos(POSITIONS.L4));
        operatorController.start().onTrue(elevator.moveToPosition());

        operatorController.leftTrigger().whileTrue(climber.runClimber(-CLIMBER.speed));
        operatorController.leftTrigger().whileTrue(climber.runClimber(CLIMBER.speed));

        operatorController.leftBumper().onTrue(coralEndEffector.intakeCoral());
        operatorController.rightBumper().onTrue(coralEndEffector.outtakeCoral());

        //operatorController.povUp().onTrue();
    }

    /**
    * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be
    * called on robot disable to prevent integral windup.
    */
    public void disablePIDSubsystems() {
        elevator.disable();
        coralPivot.disable();
        algaePivot.disable();
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}