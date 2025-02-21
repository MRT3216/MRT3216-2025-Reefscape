// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MechanismVisualization;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ElevatorConstants;
import frc.robot.settings.Constants.SimulationConstants;

public class MechanismVisualizationSubsystem extends SubsystemBase {

    private static MechanismVisualizationSubsystem m_instance;

    private Mechanism2d m_mech2d;
    private MechanismRoot2d m_mech2dRoot;

    private MechanismLigament2d m_elevatorMech2d;
    private DoubleSupplier m_elevatorHeightSupplier;

    private MechanismVisualizationSubsystem() {
        this.m_mech2d = new Mechanism2d(250, 250, new Color8Bit(255, 255, 255));
        this.m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
        this.m_elevatorMech2d = m_mech2dRoot
                .append(new MechanismLigament2d("Elevator", 0.5, 90));

        MechanismRoot2d secondaryRoot2d = m_mech2d.getRoot("Riser Root", 20, 0);
        secondaryRoot2d.append(new MechanismLigament2d("Elevator Riser",
                ElevatorConstants.kMaxElevatorHeightMeters * SimulationConstants.kVisualizationPixelMultiplier, 90, 3,
                new Color8Bit(Color.kRed)));

        SmartDashboard.putData("Robot Sim", m_mech2d);

    }

    public void registerElevatorHeightSupplier(DoubleSupplier elevatorHeightSupplier) {
        m_elevatorHeightSupplier = elevatorHeightSupplier;
        m_elevatorMech2d.setLength(m_elevatorHeightSupplier.getAsDouble());
    }

    public static MechanismVisualizationSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new MechanismVisualizationSubsystem();
        }
        return m_instance;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run
        m_elevatorMech2d
                .setLength(m_elevatorHeightSupplier.getAsDouble() * SimulationConstants.kVisualizationPixelMultiplier);
    }
}