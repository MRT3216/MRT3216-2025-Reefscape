package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.RobotMap.ROBOT.CLIMBER_MAP;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX motorController;

    public ClimberSubsystem() {
        motorController = new TalonFX(CLIMBER_MAP.motorCANID);

        // TODO: Need to finish this configuration
        var talonFXConfigurator = motorController.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        talonFXConfigurator.apply(config);

    }

    public Command runClimber(double speed) {
        return this.startEnd(
                () -> setSpeed(speed),
                () -> stop());
    }

    private void setSpeed(double speed) {
        motorController.set(speed);
    }

    private void stop() {
        motorController.set(0);
    }
}