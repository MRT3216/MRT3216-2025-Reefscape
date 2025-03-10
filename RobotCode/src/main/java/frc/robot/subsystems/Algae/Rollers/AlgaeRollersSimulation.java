package frc.robot.subsystems.Algae.Rollers;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Sim.BatterySim.BatterySimSubsystem;

public class AlgaeRollersSimulation {
    private BatterySimSubsystem m_simBattery;
    private DCMotor rollersGearbox;
    private DCMotorSim flywheelSim;

    private TalonFX realMotorController;

    public AlgaeRollersSimulation(TalonFX motorController) {
        this.rollersGearbox = DCMotor.getKrakenX60Foc(1);

        flywheelSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        rollersGearbox, 0.001, 25),
                rollersGearbox);

        this.realMotorController = motorController;
        this.m_simBattery = BatterySimSubsystem.getInstance();
    }

    public void simulationPeriodic() {
        var talonFXSim = realMotorController.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        flywheelSim.setInputVoltage(motorVoltage.in(Volts));
        flywheelSim.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(flywheelSim.getAngularPosition().times(25));
        talonFXSim.setRotorVelocity(flywheelSim.getAngularVelocity());
        m_simBattery.addCurrent(flywheelSim.getCurrentDrawAmps());
        SmartDashboard.putNumber("Algae Rollers Sim Current Draw", flywheelSim.getCurrentDrawAmps());
    }
}
