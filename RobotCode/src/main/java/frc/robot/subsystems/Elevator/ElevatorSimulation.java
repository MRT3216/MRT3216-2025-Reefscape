// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Kilograms;
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
import frc.robot.settings.Constants.Coral.ElevatorConstants;
import frc.robot.settings.Constants.SimulationConstants;
import frc.robot.subsystems.BatterySim.BatterySimSubsystem;
import frc.robot.subsystems.MechanismVisualization.MechanismVisualizationSubsystem;

public class ElevatorSimulation {
    private BatterySimSubsystem simBattery;

    private DCMotor elevatorGearbox;
    private ElevatorSim elevatorSim;

    private RelativeEncoder realEncoder;
    private SparkRelativeEncoderSim simEncoder;

    private SparkFlex realMotorController;
    private SparkFlexSim simMotorController;

    public ElevatorSimulation(RelativeEncoder encoder, SparkFlex motorController) {
        this.elevatorGearbox = DCMotor.getNeoVortex(2);
        this.elevatorSim = new ElevatorSim(elevatorGearbox, ElevatorConstants.kElevatorGearing,
                ElevatorConstants.kCarriageMass.in(Kilograms), ElevatorConstants.kElevatorDrumRadius,
                ElevatorConstants.kMinElevatorHeight.in(Meters),
                ElevatorConstants.kMaxElevatorHeight.in(Meters), true, 0);

        this.realMotorController = motorController;
        this.simMotorController = new SparkFlexSim(motorController, DCMotor.getNeoVortex(2));
        this.realEncoder = encoder;
        this.simEncoder = new SparkRelativeEncoderSim(realMotorController);
        this.simBattery = BatterySimSubsystem.getInstance();

        MechanismVisualizationSubsystem.getInstance().registerElevatorHeightSupplier(realEncoder::getPosition);
    }

    protected void simulationPeriodic() {
        elevatorSim.setInput(simMotorController.getAppliedOutput() * RobotController.getBatteryVoltage());
        elevatorSim.update(SimulationConstants.kSimulationTimeStep);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        simMotorController.iterate(
                ElevatorSimulation.convertDistanceToRotations(Meters.of(elevatorSim.getVelocityMetersPerSecond()))
                        .per(Second)
                        .in(RPM),
                RoboRioSim.getVInVoltage(),
                0.020);

        simEncoder.setPosition(elevatorSim.getPositionMeters());
        simBattery.addCurrent(elevatorSim.getCurrentDrawAmps());

        SmartDashboard.putNumber("Elevator Sim Current Draw", elevatorSim.getCurrentDrawAmps());
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