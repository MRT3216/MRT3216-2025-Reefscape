// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Sim.MechanismVisualization;

import static edu.wpi.first.units.Units.Degrees;
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
import frc.robot.settings.Constants.ALGAE;
import frc.robot.settings.Constants.CORAL;
import frc.robot.settings.Constants.CORAL.ELEVATOR;
import frc.robot.settings.Constants.SIMULATION;

public class MechanismVisualizationSubsystem extends SubsystemBase {
    // #region Fields

    private static MechanismVisualizationSubsystem m_instance;

    private Mechanism2d m_mech2d;
    private MechanismRoot2d elevatorMech2dRoot;
    private MechanismRoot2d algaePivotMech2dRoot;

    private MechanismLigament2d m_elevatorMech2d;
    private DoubleSupplier m_elevatorHeightSupplier;

    private MechanismLigament2d coralPivotMech2d;
    private DoubleSupplier coralPivotAngleSupplier;

    private MechanismLigament2d algaePivotMech2d;
    private DoubleSupplier algaePivotAngleSupplier;

    // #endregion

    private MechanismVisualizationSubsystem() {
        this.m_mech2d = new Mechanism2d(250, 250, new Color8Bit(255, 255, 255));
        this.elevatorMech2dRoot = m_mech2d.getRoot("Elevator Root", 100, 50);
        this.m_elevatorMech2d = elevatorMech2dRoot
                .append(new MechanismLigament2d("Elevator", 0.5, 90));

        MechanismRoot2d secondaryRoot2d = m_mech2d.getRoot("Riser Root", 120, 50);
        secondaryRoot2d.append(new MechanismLigament2d("Elevator Riser",
                ELEVATOR.kMaxHeight.in(Meters) * SIMULATION.kVisualizationPixelMultiplier,
                90,
                3,
                new Color8Bit(Color.kRed)));

        this.coralPivotMech2d = this.m_elevatorMech2d.append(
                new MechanismLigament2d("Coral Pivot",
                        CORAL.PIVOT.kPivotArmLength.in(Meters)
                                * SIMULATION.kVisualizationPixelMultiplier,
                        // -90 to compensate for the elevator's 90 degree rotation
                        CORAL.PIVOT.kMinPivotAngle.in(Degrees),
                        10,
                        new Color8Bit(Color.kPurple)));

        this.algaePivotMech2dRoot = m_mech2d.getRoot("Algae Pivot Root", 50, 50);
        this.algaePivotMech2d = this.algaePivotMech2dRoot.append(
                new MechanismLigament2d("Algae Pivot",
                        ALGAE.PIVOT.kPivotArmLength.in(Meters)
                                * SIMULATION.kVisualizationPixelMultiplier,
                        ALGAE.PIVOT.kMinPivotAngle.in(Degrees),
                        10,
                        new Color8Bit(Color.kBlack)));

        SmartDashboard.putData("Robot Sim", m_mech2d);
    }

    public static MechanismVisualizationSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new MechanismVisualizationSubsystem();
        }
        return m_instance;
    }

    public void registerElevatorHeightSupplier(DoubleSupplier elevatorHeightSupplier) {
        m_elevatorHeightSupplier = elevatorHeightSupplier;
    }

    public void registerCoralPivotAngleSupplier(DoubleSupplier pivotAngleSupplier) {
        coralPivotAngleSupplier = pivotAngleSupplier;
    }

    public void registerAlgaePivotAngleSupplier(DoubleSupplier pivotAngleSupplier) {
        algaePivotAngleSupplier = pivotAngleSupplier;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run
        m_elevatorMech2d
                .setLength(m_elevatorHeightSupplier.getAsDouble()
                        * SIMULATION.kVisualizationPixelMultiplier);

        coralPivotMech2d
                .setAngle(Units.rotationsToDegrees(coralPivotAngleSupplier.getAsDouble()));

        algaePivotMech2d
                // Make the angle negative and mirror it by adding 180 so it moves to the left
                .setAngle(Units.rotationsToDegrees(-algaePivotAngleSupplier.getAsDouble()) + 180);
    }
}