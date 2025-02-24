package frc.robot.subsystems.Coral;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.CoralIntakeConstants;

public class CoralIntakeSubsystem extends SubsystemBase{
    private final TalonFX intakeMotor;

    public CoralIntakeSubsystem() {
        intakeMotor = new TalonFX(0);

        // TalonFXConfiguration config = new TalonFXConfiguration();
        // TalonFXConfigurator configurer = new TalonFXConfigurator(intakeMotor);
        // TODO: Add configuration settings (Configuration vs. Configurator?)
        
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    //TODO: check this again
    public Command runIntakeCommand(double speed) {
        double voltage = 0; // intakeMotor.getSupllyVoltage();
        return this.startEnd(
                () -> setIntakeSpeed(speed), 
                () -> stopIntake())
                .until(() -> voltage > CoralIntakeConstants.VOLTAGE_THRESHOLD);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }
}
