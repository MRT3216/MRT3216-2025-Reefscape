// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.settings.Constants.ElevatorConstants;
import frc.robot.settings.Constants.SimulationConstants;
import frc.robot.subsystems.BatterySim.BatterySimSubsystem;
import frc.robot.subsystems.MechanismVisualization.MechanismVisualizationSubsystem;

public class ElevatorSimulation {
    private BatterySimSubsystem m_simBattery;

    private DCMotor m_elevatorGearbox;
    private ElevatorSim m_elevatorSim;

    private RelativeEncoder m_realEncoder;
    private SparkRelativeEncoderSim m_simEncoder;

    private SparkFlex m_realMotorController;
    private SparkFlexSim m_simMotorController;

    public ElevatorSimulation(RelativeEncoder encoder, SparkFlex motorController) {
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
        m_elevatorSim.setInput(m_simMotorController.getAppliedOutput() * RobotController.getBatteryVoltage());

        m_elevatorSim.update(SimulationConstants.kSimulationTimeStep);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_simMotorController.iterate(
                ElevatorSimulation.convertDistanceToRotations(Meters.of(m_elevatorSim.getVelocityMetersPerSecond()))
                        .per(Second)
                        .in(RPM),
                RoboRioSim.getVInVoltage(),
                0.020);

        m_simEncoder.setPosition(m_elevatorSim.getPositionMeters());
        m_simBattery.addCurrent(m_elevatorSim.getCurrentDrawAmps());

        SmartDashboard.putNumber("ElevatorSimCurrentDraw", m_elevatorSim.getCurrentDrawAmps());
    }

    /**
    * Convert {@link Distance} into {@link Angle}
    *
    * @param distance Distance, usually Meters.
    * @return {@link Angle} equivalent to rotations of the motor.
    */
    public static Angle convertDistanceToRotations(Distance distance) {
        return Rotations.of(distance.in(Meters) /
                (ElevatorConstants.kElevatorDrumRadius * 2 * Math.PI) *
                ElevatorConstants.kElevatorGearing);
    }
}