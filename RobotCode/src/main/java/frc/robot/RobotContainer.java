// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ComboCommands;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.ALGAE.PIVOT.Positions;
import frc.robot.settings.Constants.BranchSide;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.settings.Constants.CoralStationSide;
import frc.robot.settings.RobotMap;
import frc.robot.subsystems.Algae.Pivot.AlgaePivotSubsystem;
import frc.robot.subsystems.Algae.Rollers.AlgaeRollersSubsystem;
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
    private final ComboCommands comboCommands = new ComboCommands(drivetrain, elevator, coralPivot, coralEndEffector);

    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("None");;

    // #endregion

    public RobotContainer() {
        configureAutos();
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureAutos() {
        autoChooser.addOption("Left 3P", AutoCommands.getLeft3PAuto(comboCommands));
        autoChooser.addOption("Center 1P", AutoCommands.getCenter1PAuto(comboCommands));
        autoChooser.addOption("Right 3P", AutoCommands.getRight3PAuto(comboCommands));
        autoChooser.addOption("Drive Forward", AutoCommands.driveForward(drivetrain));
        autoChooser.addOption("Drive Forward L1",
                AutoCommands.driveForwardL1(drivetrain, elevator, coralPivot, comboCommands));
        autoChooser.addOption("Push Forward", AutoCommands.pushForward(drivetrain));
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
                                                                ? 0.15
                                                                : 0.8)) // Drive forward with negative Y (forward)
                                .withVelocityY(
                                        -driverController
                                                .getLeftX()
                                                * Constants.DRIVETRAIN.MaxSpeed
                                                * (drivetrain.isSlowMode().getAsBoolean()
                                                        || elevator.aboveHeight().getAsBoolean()
                                                                ? 0.15
                                                                : 0.8)) // Drive left with negative X (left)
                                .withRotationalRate(
                                        -driverController
                                                .getRightX()
                                                * Constants.DRIVETRAIN.MaxAngularRate
                                                * (drivetrain.isSlowMode().getAsBoolean()
                                                        || elevator.aboveHeight().getAsBoolean()
                                                                ? 0.3
                                                                : 0.8)) // Drive counterclockwise with negative X (left)
                ));

        driverController.a().whileTrue(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.LEFT));
        driverController.b().whileTrue(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT));
        driverController.x().onTrue(comboCommands.scoreCoral());
        driverController.y().whileTrue(DriveCommands.driveToProcessor(drivetrain));

        driverController.leftTrigger()
                .whileTrue(comboCommands.driveToNearestReefThenAlignAndScorePrep(() -> BranchSide.LEFT));

        driverController.rightTrigger()
                .whileTrue(comboCommands.driveToNearestReefThenAlignAndScorePrep(() -> BranchSide.RIGHT));

        // driverController.rightTrigger().onTrue(
        //         CoralCommands.moveElevatorAndPivotToHeightCommandDelayPivot(elevator,
        //                 coralPivot, elevator.getSelectedPosition()));

        // driverController.leftBumper().onTrue(AlgaeCommands.intakeAlgae(algaePivot, algaeRollers));
        // driverController.rightBumper().onTrue(AlgaeCommands.scoreAlgae(algaePivot, algaeRollers));

        // Reset the field-centric heading on start press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.leftStick().onTrue(
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot, POSITIONS.STOW));
        driverController.rightStick().onTrue(drivetrain.toggleSlowMode());

        // #region Testing

        driverController.povDown().onTrue(elevator.setTargetPos(POSITIONS.L1));
        driverController.povLeft().onTrue(elevator.setTargetPos(POSITIONS.L2));
        driverController.povRight().onTrue(elevator.setTargetPos(POSITIONS.L3));
        driverController.povUp().onTrue(elevator.setTargetPos(POSITIONS.L4));

        // TODO: Use this method to aim wheels for climb
        // driverController.b().whileTrue(drivetrain.applyRequest(
        //         () -> point.withModuleDirection(
        //                 new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        // driverController.povUp()
        //         .onTrue(CoralCommands.moveElevatorAndPivotToHeightCommandDelayPivot(elevator, coralPivot,
        //                 () -> POSITIONS.L4));
        // driverController.povRight()
        //         .onTrue(CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
        //                 () -> POSITIONS.L3));
        // driverController.povLeft()
        //         .onTrue(CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
        //                 () -> POSITIONS.L2));
        // driverController.povDown()
        //         .onTrue(CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
        //                 () -> POSITIONS.CORAL_STATION));
        // driverController.back()
        //         .onTrue(CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
        //                 () -> POSITIONS.SCORE_PREP));

        driverController.leftBumper().whileTrue(coralPivot.adjustPivotAngle(Degrees.of(-1)).repeatedly());
        driverController.rightBumper().whileTrue(coralPivot.adjustPivotAngle(Degrees.of(1)).repeatedly());
        // driverController.leftBumper().onTrue(algaePivot.adjustPivotAngle(Degrees.of(-1)));
        // driverController.rightBumper().onTrue(algaePivot.adjustPivotAngle(Degrees.of(1)));

        // driverController.leftTrigger().whileTrue(elevator.adjustElevatorHeight(Inches.of(-0.5)).repeatedly());
        // driverController.rightTrigger().whileTrue(elevator.adjustElevatorHeight(Inches.of(0.5)).repeatedly());
        // driverController.leftTrigger().whileTrue(climber.runClimber(-0.3));
        // driverController.rightTrigger().whileTrue(climber.runClimber(0.3));

        // #endregion

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureOperatorBindings() {
        operatorController.a().onTrue(elevator.setTargetPos(POSITIONS.L1));
        operatorController.b().onTrue(elevator.setTargetPos(POSITIONS.L2));
        operatorController.x().onTrue(elevator.setTargetPos(POSITIONS.L3));
        operatorController.y().onTrue(elevator.setTargetPos(POSITIONS.L4));
        operatorController.start().onTrue(
                CoralCommands.moveElevatorAndPivotToHeightCommandDelayPivot(elevator,
                        coralPivot, elevator.getTargetPosition()));

        operatorController.leftStick().onTrue(
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot, POSITIONS.STOW));

        // operatorController.leftTrigger().whileTrue(climber.runClimber(-CLIMBER.speed));
        // operatorController.rightTrigger().whileTrue(climber.runClimber(CLIMBER.speed));
        operatorController.leftTrigger().onTrue(algaePivot.movePivotToAngle(Positions.INTAKING));
        operatorController.rightTrigger().onTrue(algaePivot.movePivotToAngle(Positions.STOW_SCORING));
        operatorController.leftBumper().onTrue(coralEndEffector.intakeCoralCommand());
        operatorController.rightBumper().onTrue(coralEndEffector.outtakeCoralCommand());
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

    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Slow Mode",
                drivetrain.isSlowMode().getAsBoolean() || elevator.aboveHeight().getAsBoolean());
    }
}