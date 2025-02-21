// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorSim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/** Add your docs here. */
public final class ElevatorCommandFactory {

    private ElevatorSubsystem m_elevatorSubsystem;

    public ElevatorCommandFactory(ElevatorSubsystem elevatorSubsystem) {
        this.m_elevatorSubsystem = elevatorSubsystem;
    }

    private Command addStart(Command command) {
        Command fullCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_elevatorSubsystem.enable()),
                command);

        return fullCommand;
    }

    private Command addRequirement(Command command) {
        command.addRequirements(m_elevatorSubsystem);
        return command;
    }

    public Command createMoveElevatorCommand(double goal) {
        return new InstantCommand(() -> m_elevatorSubsystem.setGoal(goal))
                .andThen(new WaitUntilCommand(() -> m_elevatorSubsystem.atGoal()));
    }

    public Command createMoveUpAndDownCommand() {
        System.out.println("Creating move up and down command");
        Command command = new SequentialCommandGroup(
                createMoveElevatorCommand(1.5),
                createMoveElevatorCommand(0.5));

        command = addStart(command);
        return addRequirement(command);
    }
}
