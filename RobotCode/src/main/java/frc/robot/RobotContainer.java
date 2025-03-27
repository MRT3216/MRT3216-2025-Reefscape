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
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ComboCommands;
import frc.robot.commands.CoralCommands;
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

    // TODO: reset this to stow
    private POSITIONS targetPosition = POSITIONS.STOW;

    // #endregion

    public RobotContainer() {
        configureAutos();
        configureDriverBindings();
        configureOperatorBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureAutos() {
        autoChooser.addOption("Left 3P", AutoCommands.getLeft3PAuto(comboCommands));
        autoChooser.addOption("Right 3P", AutoCommands.getRight3PAuto(comboCommands));
        autoChooser.addOption("Center 1P", AutoCommands.getCenter1PAuto(comboCommands));
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

        driverController.x().whileTrue(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.LEFT));
        driverController.y().whileTrue(comboCommands.retrieveFromCoralStationCommand(() -> CoralStationSide.RIGHT));
        driverController.a().onTrue(comboCommands.scoreCoral());
        driverController.b().onTrue(comboCommands.intakeCoralFromStationCommand());

        driverController.leftTrigger()
                .whileTrue(comboCommands.driveToNearestReefThenAlignAndScorePrep(() -> this.targetPosition,
                        () -> BranchSide.LEFT));

        driverController.rightTrigger()
                .whileTrue(comboCommands.driveToNearestReefThenAlignAndScorePrep(() -> this.targetPosition,
                        () -> BranchSide.RIGHT));

        driverController.leftBumper().onTrue(AlgaeCommands.intakeAlgae(algaePivot, algaeRollers));
        driverController.rightBumper().onTrue(AlgaeCommands.scoreAlgae(algaePivot, algaeRollers));

        // Reset the field-centric heading on start press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.leftStick().onTrue(
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot, POSITIONS.STOW));
        driverController.rightStick().onTrue(drivetrain.toggleSlowMode());

        // #region Testing

        driverController.povDown().onTrue(//this.setTargetPos(POSITIONS.L1));
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot, POSITIONS.L1));
        driverController.povLeft().onTrue(//this.setTargetPos(POSITIONS.L2));
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot, POSITIONS.L2));
        driverController.povRight().onTrue(//this.setTargetPos(POSITIONS.L3));
                elevator.moveElevatorToPosition(POSITIONS.L3));
        driverController.povUp().onTrue(//this.setTargetPos(POSITIONS.L4));
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot, POSITIONS.L4));

        // driverController.leftBumper().whileTrue(algaePivot.adjustPivotAngle(Degrees.of(-0.5)).repeatedly());
        // driverController.rightBumper().whileTrue(algaePivot.adjustPivotAngle(Degrees.of(0.5)).repeatedly());

        // driverController.leftTrigger().whileTrue(elevator.adjustElevatorHeight(Inches.of(-0.5)).repeatedly());
        // driverController.rightTrigger().whileTrue(elevator.adjustElevatorHeight(Inches.of(0.5)).repeatedly());
        // driverController.leftBumper().whileTrue(coralPivot.adjustPivotAngle(Degrees.of(-1)).repeatedly());
        // driverController.rightBumper().whileTrue(coralPivot.adjustPivotAngle(Degrees.of(1)).repeatedly());

        // elevator.setDefaultCommand(elevator.runManualElevator(() -> driverController.getRightTriggerAxis() * .15));

        // #endregion
    }

    private void configureOperatorBindings() {
        operatorController.a().onTrue(this.setTargetPos(POSITIONS.L1));
        operatorController.b().onTrue(this.setTargetPos(POSITIONS.L2));
        operatorController.x().onTrue(this.setTargetPos(POSITIONS.L3));
        operatorController.y().onTrue(this.setTargetPos(POSITIONS.L4));
        operatorController.start().onTrue(
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator,
                        coralPivot, this.targetPosition));

        operatorController.leftStick().onTrue(
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot, POSITIONS.STOW));

        operatorController.leftTrigger().onTrue(algaePivot.movePivotToAngle(Positions.INTAKING));
        operatorController.rightTrigger().onTrue(algaePivot.movePivotToAngle(Positions.STOW_SCORING));
        operatorController.leftBumper().onTrue(coralEndEffector.intakeCoralCommand());
        operatorController.rightBumper().onTrue(coralEndEffector.outtakeCoralCommand());

        operatorController.povDown().onTrue(
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
                        POSITIONS.LOWER_ALGAE_PREP));
        operatorController.povLeft().onTrue(
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
                        POSITIONS.LOWER_ALGAE_REMOVE));
        operatorController.povRight().onTrue(
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
                        POSITIONS.UPPER_ALGAE_REMOVE));
        operatorController.povUp().onTrue(
                CoralCommands.moveElevatorAndPivotToHeightCommand(elevator, coralPivot,
                        POSITIONS.UPPER_ALGAE_PREP));
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

    private Command setTargetPos(POSITIONS pos) {
        return Commands.runOnce(() -> targetPosition = pos);
    }

    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Slow Mode",
                drivetrain.isSlowMode().getAsBoolean() || elevator.aboveHeight().getAsBoolean());

        SmartDashboard.putBoolean("L4", targetPosition == POSITIONS.L4);
        SmartDashboard.putBoolean("L3", targetPosition == POSITIONS.L3);
        SmartDashboard.putBoolean("L2", targetPosition == POSITIONS.L2);
        SmartDashboard.putBoolean("L1", targetPosition == POSITIONS.L1);
        SmartDashboard.putBoolean("Stow", targetPosition == POSITIONS.STOW);
        SmartDashboard.putBoolean("Coral Station", targetPosition == POSITIONS.CORAL_STATION);
        SmartDashboard.putBoolean("Score Prep", targetPosition == POSITIONS.SCORE_PREP);
    }
}