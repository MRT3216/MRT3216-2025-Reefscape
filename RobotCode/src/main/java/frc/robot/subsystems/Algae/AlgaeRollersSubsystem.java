package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ALGAE.PIVOT;
import frc.robot.settings.Constants.CORAL.END_EFFECTOR;
import frc.robot.settings.RobotMap.ROBOT.ALGAE_SYSTEM.ROLLERS_MAP;

public class AlgaeRollersSubsystem extends SubsystemBase {
    private final SparkMax motorController;

    public AlgaeRollersSubsystem() {
        motorController = new SparkMax(ROLLERS_MAP.motorCANID, MotorType.kBrushless);

        // TODO: Need to finish this configuration
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake)
                .inverted(PIVOT.kMotorInverted)
                .smartCurrentLimit(PIVOT.kMotorCurrentLimit)
                .voltageCompensation(PIVOT.kVoltageCompensation)
                .openLoopRampRate(PIVOT.kPivotRampRate);
    }

    private void setIntakeSpeed(double speed) {
        motorController.set(speed);
    }

    private void stopIntake() {
        motorController.set(0);
    }

    // TODO: check this again
    public Command runRollerCommand(double speed) {
        double voltage = 0;
        return this.startEnd(
                () -> setIntakeSpeed(speed),
                () -> stopIntake());
                // // TODO: I think this neds more logic (check last years bot)
                // .until(() -> voltage > END_EFFECTOR.VOLTAGE_THRESHOLD);
    }
}