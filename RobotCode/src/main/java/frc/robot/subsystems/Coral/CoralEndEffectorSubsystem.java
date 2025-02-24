package frc.robot.subsystems.Coral;

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

        TalonFXConfigurator config = motor.getConfigurator();


    }

    public void setIntakeSpeed(double speed) {
        motor.set(speed);
    }

    //TODO: check this again
    public Command runIntakeCommand(double speed) {
        double voltage = 0; // intakeMotor.getSupllyVoltage();
        return this.startEnd(
                () -> setIntakeSpeed(speed),
                () -> stopIntake())
                .until(() -> voltage > END_EFFECTOR.VOLTAGE_THRESHOLD);
    }

    public void stopIntake() {
        motor.set(0);
    }
}
