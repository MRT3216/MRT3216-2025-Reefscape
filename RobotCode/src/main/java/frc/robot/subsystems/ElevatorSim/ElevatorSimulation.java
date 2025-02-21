// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorSim;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.settings.Constants.ElevatorConstants;
import frc.robot.settings.Constants.SimulationConstants;
import frc.robot.subsystems.BatterySim.BatterySimSubsystem;
import frc.robot.subsystems.MechanismVisualization.MechanismVisualizationSubsystem;

public class ElevatorSimulation {
    private BatterySimSubsystem m_simBattery;

    private DCMotor m_elevatorGearbox;
    private ElevatorSim m_elevatorSim;

    private SparkRelativeEncoder m_realEncoder;
    private SparkRelativeEncoderSim m_simEncoder;

    private SparkFlex m_realMotorController;
    private SparkFlexSim m_simMotorController;

    public ElevatorSimulation(SparkRelativeEncoder encoder, SparkFlex motorController) {
        this.m_elevatorGearbox = DCMotor.getNeoVortex(2);
        this.m_elevatorSim = new ElevatorSim(m_elevatorGearbox, ElevatorConstants.kElevatorGearing,
                ElevatorConstants.kCarriageMass, ElevatorConstants.kElevatorDrumRadius,
                ElevatorConstants.kMinElevatorHeightMeters, ElevatorConstants.kMaxElevatorHeightMeters, true, 0);

        this.m_realMotorController = motorController;
        this.m_simMotorController = new SparkFlexSim(motorController, DCMotor.getNeoVortex(2));
        this.m_realEncoder = encoder;
        this.m_simEncoder = new SparkRelativeEncoderSim(m_realMotorController);
        this.m_simBattery = BatterySimSubsystem.getInstance();

        MechanismVisualizationSubsystem.getInstance().registerElevatorHeightSupplier(m_realEncoder::getPosition);
    }

    protected void simulationPeriodic() {
        m_elevatorSim.setInput(m_simMotorController.getVelocity() * RobotController.getBatteryVoltage());

        m_elevatorSim.update(SimulationConstants.kSimulationTimeStep);

        m_simEncoder.setPosition(m_elevatorSim.getPositionMeters());

        m_simBattery.addCurrent(m_elevatorSim.getCurrentDrawAmps());

        SmartDashboard.putNumber("ElevatorSimCurrentDraw", m_elevatorSim.getCurrentDrawAmps());
    }
}
