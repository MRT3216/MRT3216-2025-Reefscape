package frc.robot.subsystems.Coral;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.settings.Constants.CoralPivotConstants;
import frc.robot.settings.Constants.SimulationConstants;
import frc.robot.subsystems.BatterySim.BatterySimSubsystem;
import frc.robot.subsystems.MechanismVisualization.MechanismVisualizationSubsystem;

public class CoralPivotSimulation {
    private BatterySimSubsystem m_simBattery;

    private DCMotor pivotGearbox;
    private SingleJointedArmSim armPivotSim;

    private AbsoluteEncoder realEncoder;
    private SparkAbsoluteEncoderSim simEncoder;

    private SparkFlex realMotorController;
    private SparkFlexSim simMotorController;

    public CoralPivotSimulation(AbsoluteEncoder encoder, SparkFlex motorController) {
        this.pivotGearbox = DCMotor.getNeoVortex(1);
        this.armPivotSim = new SingleJointedArmSim(
                pivotGearbox,
                CoralPivotConstants.kPivotGearing,
                CoralPivotConstants.kMOI,
                CoralPivotConstants.kPivotArmLength.in(Meters),
                CoralPivotConstants.kMinPivotAngle.in(Radians),
                CoralPivotConstants.kMaxPivotAngle.in(Radians),
                true,
                Units.degreesToRadians(0));

        this.realMotorController = motorController;
        this.simMotorController = new SparkFlexSim(realMotorController, pivotGearbox);
        this.realEncoder = encoder;
        this.simEncoder = new SparkAbsoluteEncoderSim(realMotorController);
        this.m_simBattery = BatterySimSubsystem.getInstance();

        MechanismVisualizationSubsystem.getInstance()
                .registerCoralPivotAngleSupplier(
                        () -> realEncoder.getPosition() + Units.degreesToRadians(180));
    }

    protected void simulationPeriodic() {
        armPivotSim.setInput(simMotorController.getAppliedOutput() * RobotController.getBatteryVoltage());
        armPivotSim.update(SimulationConstants.kSimulationTimeStep);

        // Now, we update the Spark Flex
        simMotorController.iterate(
                Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
                        armPivotSim.getVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                0.02); // Time interval, in Seconds

        simEncoder.setPosition(armPivotSim.getAngleRads());
        m_simBattery.addCurrent(armPivotSim.getCurrentDrawAmps());
        SmartDashboard.putNumber("Coral Pivot Sim Current Draw", armPivotSim.getCurrentDrawAmps());
    }
}
