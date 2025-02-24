package frc.robot.subsystems.Coral;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Coral.EndEffectorConstants;
import frc.robot.settings.RobotMap.ROBOT.CoralSystem.EndEffectorMap;

public class CoralEndEffectorSubsystem extends SubsystemBase {
    private final TalonFX motor;

    public CoralEndEffectorSubsystem() {
        motor = new TalonFX(EndEffectorMap.motorCANID);

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
                .until(() -> voltage > EndEffectorConstants.VOLTAGE_THRESHOLD);
    }

    public void stopIntake() {
        motor.set(0);
    }
}
