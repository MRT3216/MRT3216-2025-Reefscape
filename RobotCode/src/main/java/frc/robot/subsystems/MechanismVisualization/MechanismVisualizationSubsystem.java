// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MechanismVisualization;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.CoralPivotConstants;
import frc.robot.settings.Constants.ElevatorConstants;
import frc.robot.settings.Constants.SimulationConstants;

public class MechanismVisualizationSubsystem extends SubsystemBase {

    private static MechanismVisualizationSubsystem m_instance;

    private Mechanism2d m_mech2d;
    private MechanismRoot2d m_mech2dRoot;

    private MechanismLigament2d m_elevatorMech2d;
    private DoubleSupplier m_elevatorHeightSupplier;

    private MechanismLigament2d m_armMech2d;
    private DoubleSupplier m_armAngleSupplier;

    private MechanismVisualizationSubsystem() {
        this.m_mech2d = new Mechanism2d(250, 250, new Color8Bit(255, 255, 255));
        this.m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 50, 0);
        this.m_elevatorMech2d = m_mech2dRoot
                .append(new MechanismLigament2d("Elevator", 0.5, 90));

        MechanismRoot2d secondaryRoot2d = m_mech2d.getRoot("Riser Root", 60, 0);
        secondaryRoot2d.append(new MechanismLigament2d("Elevator Riser",
                ElevatorConstants.kMaxElevatorHeight.in(Meters) * SimulationConstants.kVisualizationPixelMultiplier, 180,
                3,
                new Color8Bit(Color.kRed)));

        this.m_armMech2d = this.m_elevatorMech2d.append(
                new MechanismLigament2d("Coral Pivot",
                        CoralPivotConstants.kPivotArmLength.in(Meters)
                                * SimulationConstants.kVisualizationPixelMultiplier, 90));
                        //-1 * (90 - 30)));

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

    public void registerCoralPivotAngleSupplier(DoubleSupplier armAngleSupplier) {
        m_armAngleSupplier = armAngleSupplier;
        m_armMech2d.setAngle(armAngleSupplier.getAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run
        m_elevatorMech2d
                .setLength(m_elevatorHeightSupplier.getAsDouble() * SimulationConstants.kVisualizationPixelMultiplier);

        m_armMech2d.setAngle(Units.radiansToDegrees(m_armAngleSupplier.getAsDouble()));
    }
}