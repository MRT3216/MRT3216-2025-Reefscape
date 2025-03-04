package frc.robot.subsystems.Coral;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.CORAL.END_EFFECTOR;
import frc.robot.settings.RobotMap.ROBOT.CORAL_SYSTEM.END_EFFECTOR_MAP;

public class CoralEndEffectorSubsystem extends SubsystemBase {
    private final TalonFX motor;

    public CoralEndEffectorSubsystem() {
        motor = new TalonFX(END_EFFECTOR_MAP.motorCANID);

        // TODO: Need to finish this configuration
        TalonFXConfigurator motorConfigurator = motor.getConfigurator();
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfigurator.apply(motorConfiguration);
    }

    private void setIntakeSpeed(double speed) {
        motor.set(speed);
    }

    private void stopIntake() {
        motor.set(0);
    }

    // TODO: check this again
    public Command runEndEffectorCommand(double speed) {
        double voltage = 0;
        return this.startEnd(
                () -> setIntakeSpeed(speed),
                () -> stopIntake())
                // TODO: I think this neds more logic (check last years bot)
                .until(() -> voltage > END_EFFECTOR.VOLTAGE_THRESHOLD);
    }
}