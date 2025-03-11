// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Sim.BatterySim;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BatterySimSubsystem extends SubsystemBase {

    private static BatterySimSubsystem m_instance;
    private LinkedList<Double> m_currentDraws;

    public static BatterySimSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new BatterySimSubsystem();
        }
        return m_instance;
    }

    /** Creates a new BatterySimSubsystem. */
    private BatterySimSubsystem() {
        this.m_currentDraws = new LinkedList<>();
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = BatterySim
                .calculateDefaultBatteryLoadedVoltage(m_currentDraws.stream().mapToDouble(d -> d).toArray());
        RoboRioSim.setVInVoltage(batteryVoltage);

        m_currentDraws.clear();
    }

    public void addCurrent(double currentDraw) {
        this.m_currentDraws.add(currentDraw);
    }
}
